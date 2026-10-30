"""Run unitree_rl_lab's G1 velocity policy in MuJoCo, the way g1_ctrl runs it
on the robot: the same observation, 50 Hz policy and deploy.yaml gains. The
command limits, fall tilt and Passive damping come from
real_deploy_rl/config.yaml, the file the real deploy reads.

    conda activate g1
    python sim2sim.py                                   # viewer, standing
    python sim2sim.py --cmd 0.5 0 0                     # walk forward at 0.5 m/s
    python sim2sim.py --headless --seconds 10 --cmd 0.5 0 0
    python sim2sim.py --hands                           # the model with both Dex3-1 hands
"""

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from g1_velocity_policy import CONFIG_PATH, SDK_JOINT_NAMES, load_config, tilt

ASSETS = Path(__file__).resolve().parent.parent / "assets" / "g1_description"
SIM_DT = 0.002
ARMATURE = 0.01  # every joint's armature in training (unitree.py); the MJCF sets none
START_HEIGHT = 0.8  # m, the pelvis height training resets to
SETTLE = 3.0  # s left out of the summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cmd", type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("VX", "VY", "YAW"),
                        help="m/s, m/s, rad/s in the base frame; clipped to config.yaml's commands")
    parser.add_argument("--config", default=CONFIG_PATH, help="default: real_deploy_rl/config.yaml")
    parser.add_argument("--seconds", type=float, help="stop after this long (default: 10 headless, until closed)")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--hands", action="store_true", help="use g1_29dof_with_hand_rev_1_0.xml")
    args = parser.parse_args()
    seconds = args.seconds if args.seconds is not None else (10.0 if args.headless else float("inf"))

    xml = "g1_29dof_with_hand_rev_1_0.xml" if args.hands else "g1_29dof_rev_1_0.xml"
    model = mujoco.MjModel.from_xml_path(str(ASSETS / xml))
    model.opt.timestep = SIM_DT
    data = mujoco.MjData(model)

    cfg, policy = load_config(args.config)
    decimation = round(policy.step_dt / SIM_DT)
    qadr = np.array([model.jnt_qposadr[model.joint(n).id] for n in SDK_JOINT_NAMES])
    vadr = np.array([model.jnt_dofadr[model.joint(n).id] for n in SDK_JOINT_NAMES])
    act = np.array([model.actuator(n).id for n in SDK_JOINT_NAMES])
    model.dof_armature[vadr] = ARMATURE

    cmd = np.clip(args.cmd, cfg["cmd_low"], cfg["cmd_high"])
    if not np.allclose(cmd, args.cmd):
        print(f"command clipped to config.yaml's commands: {cmd.round(3).tolist()}")
    print(f"{xml}, policy every {decimation} steps of {SIM_DT} s, observation {policy.obs_dim}, "
          f"command {cmd.round(3).tolist()}")

    # start as training resets: pelvis at 0.8 m, joints at the default pose, at rest
    data.qpos[2] = START_HEIGHT
    data.qpos[qadr] = policy.default_q_sdk
    mujoco.mj_forward(model, data)

    def lowstate():
        """What g1_ctrl reads from rt/lowstate: the pelvis IMU's quaternion
        (w first) and gyro, and the motors' q and dq in SDK order. A free
        joint's qvel[3:6] is in the body frame, as the gyro is."""
        return data.qpos[3:7].copy(), data.qvel[3:6].copy(), data.qpos[qadr].copy(), data.qvel[vadr].copy()

    def base_velocity():
        """vx, vy in the yaw frame and the yaw rate, as training tracks them."""
        rot = np.empty(9)
        mujoco.mju_quat2Mat(rot, data.qpos[3:7])
        rot = rot.reshape(3, 3)
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        c, s = np.cos(yaw), np.sin(yaw)
        vx, vy = data.qvel[0], data.qvel[1]
        return np.array([c * vx + s * vy, -s * vx + c * vy, (rot @ data.qvel[3:6])[2]])

    policy.reset(*lowstate(), cmd)
    q_target = policy.default_q_sdk.copy()
    kp, kd = policy.kp, policy.kd
    fell_at = None
    log = []  # (t, vx, vy, yaw rate, pelvis height)
    peak = np.zeros(len(SDK_JOINT_NAMES))  # largest |torque| after SETTLE, Nm

    def control_step():
        """One policy step and the physics steps it holds for."""
        nonlocal q_target, kp, kd, fell_at
        quat, gyro, q, dq = lowstate()
        if fell_at is None and tilt(quat) > cfg["fall_tilt"]:
            fell_at = data.time
            kp, kd = np.zeros_like(kp), np.full_like(kd, cfg["passive_kd"])
            print(f"FELL at t={data.time:.2f} s (tilt {tilt(quat):.2f} rad): Passive, as g1_ctrl would")
        if fell_at is None:
            q_target = policy.step(quat, gyro, q, dq, cmd)
        for _ in range(decimation):
            q, dq = data.qpos[qadr], data.qvel[vadr]
            data.ctrl[act] = kp * (q_target - q) - kd * dq  # clamped by the MJCF's actuatorfrcrange
            mujoco.mj_step(model, data)
            if data.time >= SETTLE:
                np.maximum(peak, np.abs(data.actuator_force[act]), out=peak)
        log.append((data.time, *base_velocity(), data.qpos[2]))
        if len(log) % round(1.0 / policy.step_dt) == 0 and args.headless:
            t, vx, vy, wz, z = log[-1]
            print(f"t={t:5.1f}  pelvis z={z:.3f}  vx={vx:+.2f}  vy={vy:+.2f}  yaw rate={wz:+.2f}")

    if args.headless:
        while data.time < seconds and fell_at is None:
            control_step()
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time < seconds:
                start = time.monotonic()
                control_step()
                viewer.sync()
                time.sleep(max(0.0, policy.step_dt - (time.monotonic() - start)))

    rows = np.array([r for r in log if r[0] >= SETTLE])
    if len(rows):
        vx, vy, wz, z = rows[:, 1:].mean(axis=0)
        print(f"after {SETTLE:.0f} s: vx={vx:+.3f} vy={vy:+.3f} yaw rate={wz:+.3f} "
              f"(command {cmd.round(3).tolist()}), pelvis z mean {z:.3f} min {rows[:, 4].min():.3f}")
        # left and right together, legs and waist only
        joints = {}
        for name, tau in zip(SDK_JOINT_NAMES[:15], peak[:15]):
            key = name.removeprefix("left_").removeprefix("right_").removesuffix("_joint")
            joints[key] = max(joints.get(key, 0.0), tau)
        print(f"peak |torque| after {SETTLE:.0f} s, Nm: " + ", ".join(f"{k} {v:.0f}" for k, v in joints.items()))
    print("fell" if fell_at is not None else "stayed up")
    return 1 if fell_at is not None else 0


if __name__ == "__main__":
    sys.exit(main())

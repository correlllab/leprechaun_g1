"""Run sim2sim_rl's walking policy on the real G1 from the laptop, the way
unitree_rl_lab's g1_ctrl does: rt/lowstate in, rt/lowcmd out, over DDS on the
wired port. Nothing runs on the robot's own computers. The command limits,
timings and port come from config.yaml, which sim2sim_rl/sim2sim.py reads too.

Before starting:
- the robot hangs on the gantry and the operator holds Unitree's remote;
- the operator has released Unitree's controller on the remote;
- nothing else publishes rt/lowcmd (checked: the script refuses to start).

    conda activate g1
    python deploy_real.py                       # config.yaml: enp130s0, DDS domain 0
    python deploy_real.py --zero-command        # sticks ignored, command (0, 0, 0)

It sends nothing until the operator asks. On Unitree's remote, as with g1_ctrl:

    L2 + Up    Stand: the joints move from where they are to the policy's
               default pose over stand_seconds (from the start or from Passive)
    R1 + X     Walk: the policy starts (from Stand, once the ramp is done and
               both sticks are centred)
    L2 + B     Passive: every motor damping only (passive_kd), from any state
    left stick forward/back and sideways, right stick left/right turns; a full
    stick gives the limit in config.yaml's commands

It also goes to Passive by itself if rt/lowstate stops for state_timeout, the
pelvis tilts past fall_tilt while walking, or the policy fails. Ctrl+C sends
Passive for exit_passive_seconds, then exits; after that the motors get no
more commands.
"""

import argparse
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "sim2sim_rl"))
from g1_velocity_policy import CONFIG_PATH, load_config, tilt  # noqa: E402

NUM_MOTORS = 29
MODE_MACHINE = 5  # the 29-DoF G1, the policy's model


class Remote:
    """Unitree's remote, from rt/lowstate's wireless_remote bytes, in the
    layout of the SDK's example/wireless_controller/wireless_controller.py."""

    def __init__(self, raw):
        b = bytes(raw)
        self.R1 = bool(b[2] & 1 << 0)
        self.L2 = bool(b[2] & 1 << 5)
        self.B = bool(b[3] & 1 << 1)
        self.X = bool(b[3] & 1 << 2)
        self.Up = bool(b[3] & 1 << 4)
        self.lx, self.rx, self.ry, _, self.ly = struct.unpack_from("<5f", b, 4)

    def sticks_centred(self, deadzone):
        return max(abs(self.lx), abs(self.rx), abs(self.ly)) < deadzone


class LowStateReader:
    """Keeps the newest rt/lowstate and when it arrived."""

    def __init__(self):
        self.lock = threading.Lock()
        self.msg = None
        self.stamp = None
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_msg, 10)

    def _on_msg(self, msg):
        with self.lock:
            self.msg, self.stamp = msg, time.monotonic()

    def latest(self):
        with self.lock:
            return self.msg, self.stamp


def robot_state(msg):
    """The pelvis IMU's quaternion (w first) and gyro, and q and dq of the 29
    motors in SDK order, as g1_ctrl reads them."""
    q = np.array([msg.motor_state[i].q for i in range(NUM_MOTORS)])
    dq = np.array([msg.motor_state[i].dq for i in range(NUM_MOTORS)])
    return np.array(msg.imu_state.quaternion), np.array(msg.imu_state.gyroscope), q, dq


def other_lowcmd_writer(seconds=1.0):
    """True if anything publishes rt/lowcmd: two writers fight over the motors."""
    seen = []
    sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
    sub.Init(lambda msg: seen.append(1), 10)
    time.sleep(seconds)
    sub.Close()
    return bool(seen)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=CONFIG_PATH, help="default: config.yaml next to this script")
    parser.add_argument("--net", help="override robot.interface")
    parser.add_argument("--domain", type=int, help="override robot.domain_id")
    parser.add_argument("--zero-command", action="store_true",
                        help="ignore the sticks: the policy always gets (0, 0, 0), standing in place")
    parser.add_argument("--yes", action="store_true", help="skip the start-up question")
    args = parser.parse_args()

    cfg, policy = load_config(args.config)
    robot = cfg["robot"]
    net = args.net or robot["interface"]
    domain = robot["domain_id"] if args.domain is None else args.domain
    control_dt = robot["control_dt"]
    decimation = round(policy.step_dt / control_dt)
    cmd_low, cmd_high = cfg["cmd_low"], cfg["cmd_high"]
    if args.zero_command:
        cmd_low, cmd_high = np.zeros(3), np.zeros(3)
        print("--zero-command: the sticks are ignored, the command stays (0, 0, 0)")
    else:
        print(f"full stick: vx {cmd_low[0]} to {cmd_high[0]} m/s, vy {cmd_low[1]} to {cmd_high[1]} m/s, "
              f"yaw {cmd_low[2]} to {cmd_high[2]} rad/s")

    if not args.yes:
        answer = input("Robot on the gantry, Unitree's controller released on the remote, "
                       "operator holding the remote? [y/N] ")
        if answer.strip().lower() != "y":
            return 1

    ChannelFactoryInitialize(domain, net)
    if other_lowcmd_writer():
        print("Something else is publishing rt/lowcmd. Not starting: stop it first.")
        return 1

    reader = LowStateReader()
    deadline = time.monotonic() + 5.0
    while reader.latest()[0] is None:
        if time.monotonic() > deadline:
            print(f"No rt/lowstate within 5 s on {net}, domain {domain}.")
            return 1
        time.sleep(0.05)
    msg, _ = reader.latest()
    if msg.mode_machine != MODE_MACHINE:
        print(f"rt/lowstate says mode_machine {msg.mode_machine}; this policy is for {MODE_MACHINE} (29 DoF).")
        return 1
    print(f"rt/lowstate on {net}: mode_machine {msg.mode_machine}, tilt {tilt(robot_state(msg)[0]):.2f} rad")

    publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
    publisher.Init()
    crc = CRC()
    cmd_msg = unitree_hg_msg_dds__LowCmd_()

    state = "Idle"
    kp = np.zeros(NUM_MOTORS)
    kd = np.zeros(NUM_MOTORS)
    q_cmd = np.zeros(NUM_MOTORS)
    stand_start = stand_from = None
    walk_cycle = 0
    command = np.zeros(3)
    last = Remote(msg.wireless_remote)
    step_times = []
    print("Idle: sending nothing. L2 + Up to stand.")

    def enter(new_state, why=""):
        nonlocal state
        print(f"{state} -> {new_state}" + (f": {why}" if why else ""))
        state = new_state

    def publish(mode_machine):
        cmd_msg.mode_pr = 0
        cmd_msg.mode_machine = mode_machine
        for i in range(NUM_MOTORS):
            m = cmd_msg.motor_cmd[i]
            m.mode = 1
            m.q = float(q_cmd[i])
            m.dq = 0.0
            m.tau = 0.0
            m.kp = float(kp[i])
            m.kd = float(kd[i])
        cmd_msg.crc = crc.Crc(cmd_msg)
        publisher.Write(cmd_msg)

    def go_passive(why):
        nonlocal kp, kd
        if state != "Passive":
            enter("Passive", why)
        kp = np.zeros(NUM_MOTORS)
        kd = np.full(NUM_MOTORS, cfg["passive_kd"])

    next_tick = time.monotonic()
    last_report = next_tick
    try:
        while True:
            next_tick += control_dt
            msg, stamp = reader.latest()
            now = time.monotonic()
            quat, gyro, q, dq = robot_state(msg)
            remote = Remote(msg.wireless_remote)

            if now - stamp > robot["state_timeout"]:
                if state not in ("Idle", "Passive"):
                    go_passive(f"no rt/lowstate for {now - stamp:.2f} s")
            else:
                if remote.L2 and remote.B and not last.B and state != "Passive":
                    go_passive("L2 + B")
                elif remote.L2 and remote.Up and not last.Up and state in ("Idle", "Passive"):
                    enter("Stand", "L2 + Up")
                    kp, kd = policy.kp.copy(), policy.kd.copy()
                    stand_start, stand_from = now, q.copy()
                elif remote.R1 and remote.X and not last.X and state == "Stand":
                    if now - stand_start < robot["stand_seconds"]:
                        print("R1 + X ignored: still moving to the stand pose")
                    elif not remote.sticks_centred(robot["stick_deadzone"]):
                        print("R1 + X ignored: centre both sticks first")
                    else:
                        enter("Walk", "R1 + X")
                        kp, kd = policy.kp.copy(), policy.kd.copy()
                        policy.reset(quat, gyro, q, dq)
                        walk_cycle = 0
                last = remote

            if state == "Passive":
                q_cmd = q
            elif state == "Stand":
                alpha = min(1.0, (now - stand_start) / robot["stand_seconds"])
                q_cmd = (1 - alpha) * stand_from + alpha * policy.default_q_sdk
            elif state == "Walk":
                if tilt(quat) > cfg["fall_tilt"]:
                    go_passive(f"tilt {tilt(quat):.2f} rad")
                    q_cmd = q
                elif walk_cycle % decimation == 0:
                    # g1_ctrl's axes; a full stick gives the upper or the lower limit
                    sticks = np.array([remote.ly, -remote.lx, -remote.rx])
                    sticks[np.abs(sticks) < robot["stick_deadzone"]] = 0.0
                    sticks = np.clip(sticks, -1.0, 1.0)
                    command = np.where(sticks >= 0, sticks * cmd_high, -sticks * cmd_low)
                    t0 = time.perf_counter()
                    try:
                        q_cmd = policy.step(quat, gyro, q, dq, command)
                    except ValueError as e:
                        go_passive(f"policy step failed: {e}")
                        q_cmd = q
                    step_times.append(time.perf_counter() - t0)
                walk_cycle += 1
                if state == "Walk" and now - last_report >= 1.0:
                    last_report = now
                    print(f"walk: command {command.round(2).tolist()}  tilt {tilt(quat):.2f} rad  "
                          f"policy step max {1e3 * max(step_times):.1f} ms")
                    step_times.clear()

            if state != "Idle":
                publish(msg.mode_machine)

            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.05:
                print(f"control loop {-delay * 1e3:.0f} ms late")
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        why, error = "Ctrl+C", None
    except Exception as e:  # anything unexpected: damp the motors before dying
        why, error = f"error: {e!r}", e
    if state != "Idle":
        go_passive(why)
        end = time.monotonic() + robot["exit_passive_seconds"]
        while time.monotonic() < end:
            msg, _ = reader.latest()
            q_cmd = robot_state(msg)[2]
            publish(msg.mode_machine)
            time.sleep(control_dt)
    print("exiting: rt/lowcmd stops now")
    if error is not None:
        raise error
    return 0


if __name__ == "__main__":
    sys.exit(main())

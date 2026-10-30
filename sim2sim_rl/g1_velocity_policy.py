"""unitree_rl_lab's G1 velocity policy, run the way its g1_ctrl runs it.

Takes the robot's state in SDK motor order (the order of rt/lowstate's
motor_state) and returns position targets in the same order, so the MuJoCo
sim2sim and the real deploy share one copy of the observation and action code.

Ported from unitree_rl_lab/deploy at 4960b84 (Apache 2.0): observations.h,
observation_manager.h, manager_term_cfg.h, joint_actions.h and
unitree_articulation.h. Only what the velocity policy's deploy.yaml uses is
supported; anything else in the yaml is an error rather than silently ignored.
"""

from collections import deque
from pathlib import Path

import numpy as np
import onnxruntime as ort
import yaml

CONFIG_PATH = Path(__file__).resolve().parent.parent / "real_deploy_rl" / "config.yaml"
CONFIG_KEYS = {"policy_dir", "commands", "fall_tilt", "passive_kd", "robot"}
ROBOT_KEYS = {"interface", "domain_id", "control_dt", "stand_seconds", "state_timeout",
              "stick_deadzone", "exit_passive_seconds"}

# SDK motor order: slot i of rt/lowstate's motor_state and rt/lowcmd's
# motor_cmd (joint_sdk_names in unitree_rl_lab's assets/robots/unitree.py).
SDK_JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

OBS_TERMS = ["base_ang_vel", "projected_gravity", "velocity_commands",
             "joint_pos_rel", "joint_vel_rel", "last_action"]


def projected_gravity(quat_wxyz):
    """Gravity (0, 0, -1) in the body frame: conj(q) * g * q, q w first."""
    w, x, y, z = quat_wxyz
    return np.array([2 * (w * y - z * x),
                     -2 * (z * y + w * x),
                     1 - 2 * (w * w + z * z)])


def tilt(quat_wxyz):
    """Angle between the body's z axis and up, rad. g1_ctrl drops to its
    Passive state (damping only) once this passes 1.0 (bad_orientation)."""
    return float(np.arccos(np.clip(-projected_gravity(quat_wxyz)[2], -1.0, 1.0)))


def _check_keys(name, d, keys):
    if not isinstance(d, dict):
        raise ValueError(f"{name} must be a mapping")
    missing, unknown = keys - d.keys(), d.keys() - keys
    if missing or unknown:
        raise ValueError(f"{name}: missing {sorted(missing)}, unknown {sorted(unknown)}")


def load_config(path=CONFIG_PATH):
    """Load the shared config (real_deploy_rl/config.yaml) and the policy it
    names. Returns (cfg, policy), with the command limits also as arrays in
    cfg["cmd_low"] and cfg["cmd_high"] (vx, vy, yaw). Missing or unknown keys,
    and command limits beyond the policy's trained ranges, are errors."""
    path = Path(path)
    with open(path) as f:
        cfg = yaml.safe_load(f)
    _check_keys("config", cfg, CONFIG_KEYS)
    _check_keys("commands", cfg["commands"], {"vx", "vy", "yaw"})
    _check_keys("robot", cfg["robot"], ROBOT_KEYS)
    policy = VelocityPolicy((path.parent / cfg["policy_dir"]).resolve())

    limits = np.array([cfg["commands"][k] for k in ("vx", "vy", "yaw")], dtype=np.float64)
    if limits.shape != (3, 2) or not np.all(np.isfinite(limits)):
        raise ValueError("commands: vx, vy and yaw must each be [low, high]")
    low, high = limits[:, 0], limits[:, 1]
    if np.any(low > 0) or np.any(high < 0):
        raise ValueError("commands: every range must include 0")
    if np.any(low < policy.cmd_low) or np.any(high > policy.cmd_high):
        trained = np.stack([policy.cmd_low, policy.cmd_high], axis=1)
        raise ValueError(f"commands: {limits.tolist()} go beyond the policy's trained ranges {trained.tolist()}")
    if not 0 < cfg["fall_tilt"] <= np.pi / 2:
        raise ValueError("fall_tilt must be in (0, pi/2] rad")
    if cfg["passive_kd"] < 0:
        raise ValueError("passive_kd must not be negative")
    robot = cfg["robot"]
    for key in ("control_dt", "stand_seconds", "state_timeout", "stick_deadzone", "exit_passive_seconds"):
        if not robot[key] > 0:
            raise ValueError(f"robot.{key} must be positive")
    if abs(round(policy.step_dt / robot["control_dt"]) * robot["control_dt"] - policy.step_dt) > 1e-9:
        raise ValueError(f"robot.control_dt must divide the policy's {policy.step_dt} s step")
    cfg["cmd_low"], cfg["cmd_high"] = low, high
    return cfg, policy


class VelocityPolicy:
    def __init__(self, policy_dir):
        policy_dir = Path(policy_dir)
        with open(policy_dir / "params" / "deploy.yaml") as f:
            cfg = yaml.safe_load(f)

        self.step_dt = float(cfg["step_dt"])
        # policy joint i is SDK slot joint_ids_map[i]
        self.joint_ids_map = np.array(cfg["joint_ids_map"], dtype=int)
        n = len(self.joint_ids_map)
        if sorted(self.joint_ids_map) != list(range(len(SDK_JOINT_NAMES))):
            raise ValueError("joint_ids_map must cover the 29 SDK slots once each")
        # gains are in SDK order, the rest in policy order
        self.kp = np.array(cfg["stiffness"], dtype=np.float64)
        self.kd = np.array(cfg["damping"], dtype=np.float64)
        self.default_q = np.array(cfg["default_joint_pos"], dtype=np.float64)
        self.default_q_sdk = np.empty(n)
        self.default_q_sdk[self.joint_ids_map] = self.default_q

        ranges = cfg["commands"]["base_velocity"]["ranges"]
        self.cmd_low = np.array([ranges["lin_vel_x"][0], ranges["lin_vel_y"][0], ranges["ang_vel_z"][0]])
        self.cmd_high = np.array([ranges["lin_vel_x"][1], ranges["lin_vel_y"][1], ranges["ang_vel_z"][1]])

        if list(cfg["actions"]) != ["JointPositionAction"]:
            raise ValueError(f"unsupported actions: {list(cfg['actions'])}")
        action = cfg["actions"]["JointPositionAction"]
        if action["joint_ids"] is not None or action["clip"] is not None:
            raise ValueError("JointPositionAction joint_ids and clip are not supported")
        self.action_scale = np.array(action["scale"], dtype=np.float64)
        self.action_offset = np.array(action["offset"], dtype=np.float64)

        # g1_ctrl's defaults: clip before scale, each term's history
        # concatenated oldest first, terms in the yaml's order
        if list(cfg["observations"]) != OBS_TERMS:
            raise ValueError(f"expected observation terms {OBS_TERMS}, got {list(cfg['observations'])}")
        self.terms = []
        for name, term in cfg["observations"].items():
            if term["params"] and "asset_cfg" in term["params"]:
                raise ValueError(f"{name}: asset_cfg joint_ids are not supported")
            clip = None if term["clip"] is None else np.array(term["clip"], dtype=np.float64)
            scale = None if term["scale"] is None else np.array(term["scale"], dtype=np.float64)
            self.terms.append((name, clip, scale, deque(maxlen=int(term["history_length"]))))

        # one thread, no spinning: onnxruntime's default pool spins between steps
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.add_session_config_entry("session.intra_op.allow_spinning", "0")
        self.session = ort.InferenceSession(str(policy_dir / "exported" / "policy.onnx"),
                                            options, providers=["CPUExecutionProvider"])
        (inp,) = self.session.get_inputs()
        self.input_name = inp.name
        self.obs_dim = int(inp.shape[1])

        self.last_action = np.zeros(n)
        self.reset(np.array([1.0, 0, 0, 0]), np.zeros(3), self.default_q_sdk, np.zeros(n))
        if self.observation().size != self.obs_dim:
            raise ValueError(f"observation is {self.observation().size} values, the policy takes {self.obs_dim}")

    def _terms_now(self, quat_wxyz, gyro, q, dq, cmd):
        return {
            "base_ang_vel": np.asarray(gyro, dtype=np.float64),
            "projected_gravity": projected_gravity(quat_wxyz),
            "velocity_commands": np.clip(cmd, self.cmd_low, self.cmd_high),
            "joint_pos_rel": np.asarray(q)[self.joint_ids_map] - self.default_q,
            "joint_vel_rel": np.asarray(dq)[self.joint_ids_map],
            "last_action": self.last_action.copy(),
        }

    def _add(self, values):
        for name, clip, scale, history in self.terms:
            v = values[name]
            if clip is not None:
                v = np.clip(v, clip[0], clip[1])
            if scale is not None:
                v = v * scale
            history.append(v)

    def observation(self):
        return np.concatenate([np.concatenate(h) for _, _, _, h in self.terms]).astype(np.float32)

    def reset(self, quat_wxyz, gyro, q, dq, cmd=(0.0, 0.0, 0.0)):
        """Zero the last action and fill every term's history with the
        current state, as g1_ctrl does when its policy starts."""
        self.last_action = np.zeros_like(self.last_action)
        values = self._terms_now(quat_wxyz, gyro, q, dq, cmd)
        for _, _, _, history in self.terms:
            history.clear()
        for _ in range(max(h.maxlen for _, _, _, h in self.terms)):
            self._add(values)

    def step(self, quat_wxyz, gyro, q, dq, cmd):
        """One 50 Hz step. quat_wxyz and gyro are the pelvis IMU's, q and dq
        the 29 motors in SDK order, cmd (vx, vy, yaw rate) in the base frame.
        Returns the 29 position targets in SDK order."""
        values = self._terms_now(quat_wxyz, gyro, q, dq, cmd)
        if not all(np.all(np.isfinite(v)) for v in values.values()):
            raise ValueError("state is not finite")
        self._add(values)
        action = self.session.run(None, {self.input_name: self.observation()[None]})[0][0]
        self.last_action = action.astype(np.float64)
        q_target = np.empty(len(self.joint_ids_map))
        q_target[self.joint_ids_map] = self.last_action * self.action_scale + self.action_offset
        return q_target

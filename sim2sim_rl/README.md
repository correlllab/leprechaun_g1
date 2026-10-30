# sim2sim_rl

Unitree's G1 walking policy from [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab),
run in MuJoCo by a Python copy of `g1_ctrl`, the C++ controller unitree_rl_lab
ships for the real robot. The observation and action code lives in
`g1_velocity_policy.py`, which works in SDK motor order, so
[`../real_deploy_rl`](../real_deploy_rl) can drive the robot with the same file.
Both read [`../real_deploy_rl/config.yaml`](../real_deploy_rl/config.yaml): the
command limits, the fall tilt and the Passive damping, so the sim runs what the
robot runs.

## Run

It uses the `g1` conda env (Python 3.10, mujoco 3.2.3, onnxruntime, pyyaml).

```bash
conda activate g1
python sim2sim.py                                    # viewer, standing
python sim2sim.py --cmd 0.5 0 0                      # walk forward at 0.5 m/s
python sim2sim.py --headless --seconds 15 --cmd 0 0.3 0
python sim2sim.py --hands --cmd 0.5 0 0              # with both Dex3-1 hands
```

- `--cmd VX VY YAW`: m/s, m/s and rad/s in the base frame, clipped to
  `config.yaml`'s `commands` (now vx -0.5 to 0.5, vy -0.3 to 0.3, yaw -0.2 to
  0.2). The config can widen them only up to the policy's trained ranges in
  `deploy.yaml`: vx -0.5 to 1.0, vy and yaw as above.
- `--config PATH` uses another config.
- `--headless` prints the pelvis height and velocity once a second, then the
  means after the first 3 s and the peak leg and waist torques. It exits 1 if
  the robot fell.
- `--hands` loads `g1_29dof_with_hand_rev_1_0.xml`. The hands get no torque.

## Results

15 s per command, means after the first 3 s, on 2026-10-30. The robot stayed up
in every run.

| Command (vx, vy, yaw) | Measured vx, vy, yaw | Pelvis height |
|---|---|---|
| stand | 0.00, 0.00, +0.01 | 0.789 m |
| 0.5, 0, 0 | +0.49, +0.00, -0.00 | 0.783 m |
| 1.0, 0, 0 | +0.95, +0.02, +0.03 | 0.768 m |
| -0.5, 0, 0 | -0.42, -0.01, +0.02 | 0.788 m |
| 0, 0.3, 0 | +0.01, +0.25, +0.02 | 0.789 m |
| 0, -0.3, 0 | -0.01, -0.27, -0.00 | 0.789 m |
| 0, 0, 0.2 | -0.00, -0.00, +0.02 | 0.787 m |
| 0, 0, -0.2 | -0.00, +0.00, -0.01 | 0.787 m |
| 0.5, 0, 0.2 | +0.49, +0.02, +0.14 | 0.782 m |
| 1.0, 0, 0 with hands | +0.96, +0.02, +0.02 | 0.767 m |

The 1.0 m/s runs predate `config.yaml`, which now stops at 0.5.

- **It stands still for small commands** (10 s each): forward 0.1 and 0.2 m/s
  give 0.00, 0.22 gives 0.10, 0.25 gives 0.15, 0.27 gives 0.25 and 0.3 gives
  0.28; back 0.2 gives -0.11; sideways 0.15 to 0.25 gives 0.00. So on the
  robot a stick pushed only part way may not move it.
- **It barely turns on the spot.** In training only the forward and sideways
  ranges grow with the curriculum (`lin_vel_cmd_levels`), so turn commands
  stayed within ±0.1 rad/s, where a missed turn costs almost no reward. While
  walking it turns at 0.14 rad/s for 0.2.
- **Torques stay low.** The largest peaks over all runs: hip pitch 35 Nm, hip
  roll 38 Nm (sideways), knee 28 Nm, ankle pitch 21 Nm, ankle roll 11 Nm, waist
  roll 15 Nm. The ankles and waist roll and pitch stay under 25 Nm, the
  training limit, so the 35 Nm (URDF) against 50 Nm (MJCF) question in
  [`../assets/README.md`](../assets/README.md) does not matter for this policy.

## What matches g1_ctrl

- **Observation:** six terms in `deploy.yaml`'s order: pelvis angular velocity
  ×0.2, gravity in the pelvis frame, the command, the 29 joint positions minus
  the default pose, the 29 joint velocities ×0.05, and the last action. Each
  term keeps its last 5 steps, oldest first, one term after another: 480
  values.
- **Start:** every term's history is filled with the current state and the
  last action is zero.
- **Action:** 29 targets, `default + 0.25 × action`, in the policy's joint
  order; `joint_ids_map` puts policy joint i in SDK slot `joint_ids_map[i]`.
- **Rate and gains:** the policy at 50 Hz, the PD gains from `deploy.yaml`
  (legs kp 100/100/100/150/40/40, waist 200, arms 40), which are what g1_ctrl
  sends the robot.
- **Fall:** past `fall_tilt` (1 rad) the policy stops and every motor gets
  `passive_kd` (3) and no stiffness, g1_ctrl's Passive state.

## What differs

- `deploy.yaml`'s gains are not those of unitree_rl_lab's current training
  config (`assets/robots/unitree.py`): waist roll and pitch kp 200 there 40,
  arm kd 10 there 1. The v0 policy probably predates that config. We use
  `deploy.yaml`, because that is what the robot gets.
- Torque limits are the MJCF's (ankles and waist roll and pitch 50 Nm);
  training used 25 Nm.
- Every joint gets armature 0.01, as in training; the MJCF has none.
- MuJoCo steps every 2 ms with the PD computed each step. Training used
  PhysX at 5 ms with implicit PD.
- The robot starts at the default pose with its pelvis at 0.8 m, as training
  resets it. On the robot, g1_ctrl's FixStand state first moves it to the same
  pose over 3 s.
- No sensor noise and no delay.
- The policy was trained without hands.

## Files

- `g1_velocity_policy.py`: the observation, history and action, ported from
  g1_ctrl's C++ (`deploy/include/isaaclab/`), and `load_config()`, which reads
  `../real_deploy_rl/config.yaml` and refuses missing or unknown keys and
  command limits beyond the policy's trained ranges. Any `deploy.yaml` term or
  option it does not support is an error.
- `sim2sim.py`: the MuJoCo loop.
- `policy/velocity_v0/`: `exported/policy.onnx` and `params/deploy.yaml`,
  copied unchanged from unitree_rl_lab's
  `deploy/robots/g1_29dof/config/policy/velocity/v0` at commit 4960b84.
- `policy/LICENSE.unitree_rl_lab`: unitree_rl_lab's Apache 2.0 license, which
  covers the policy and the ported code.

# real_deploy_rl

Runs unitree_rl_lab's G1 walking policy on the real robot, the way Unitree's
C++ controller `g1_ctrl` does, in Python. The script runs on the laptop and
talks to the robot over DDS on the wired port: it reads `rt/lowstate` and
writes `rt/lowcmd`. Nothing runs on the robot's own computers. The policy and
its observation code live in [`../sim2sim_rl`](../sim2sim_rl), which runs the
same policy in MuJoCo with the same [`config.yaml`](config.yaml).

## Status

- **On the robot, 2026-10-30:** the policy ran on our G1 on the gantry and
  worked, with the command from the remote's sticks.
- **Offline:** tested against a fake G1 (MuJoCo behind DDS, in a private
  network namespace): the stand ramp, walking, the button checks, the
  stale-state and second-writer refusals, and valid CRCs at 500 Hz.

## What you need

- The laptop wired to the robot at `192.168.123.222/24`, and the `g1` conda
  env with `unitree_sdk2_python`: see [`../docs/bringup.md`](../docs/bringup.md).
- In that env, the policy's packages:

  ```bash
  conda activate g1
  pip install onnxruntime pyyaml
  ```

- The robot on a gantry, and someone holding Unitree's remote for the whole
  session. A second person watching the laptop helps.

## Run a session

1. **Hang the robot on the gantry**, feet off the ground.
2. **Release Unitree's controller on the remote:** from zero-torque (after
   power-on) or damping (**L2 + B**), press **L2 + R2** for debug mode. The
   joints go soft and damped.
3. **Start the script** on the laptop. `systemd-inhibit` keeps the laptop from
   sleeping mid-run:

   ```bash
   conda activate g1
   cd leprechaun_g1/real_deploy_rl
   systemd-inhibit --what=sleep:idle:handle-lid-switch --who=G1 --why="G1 RL" python deploy_real.py
   ```

   Answer `y` to the question. It should print:

   ```
   full stick: vx -0.5 to 0.5 m/s, vy -0.3 to 0.3 m/s, yaw -0.2 to 0.2 rad/s
   rt/lowstate on enp130s0: mode_machine 5, tilt 0.07 rad
   Idle: sending nothing. L2 + Up to stand.
   ```

   With the robot hanging upright the tilt should be under about 0.1 rad.
4. **L2 + Up**, still hanging: over 3 s the joints move to the policy's
   default pose (knees slightly bent, elbows bent about 55°) and hold it.
   Nothing balances yet.
5. **Lower the robot** until both feet are flat, with the gantry still taking
   the weight.
6. **Both sticks centred, R1 + X:** the policy starts and balances. Slacken the
   gantry so it only catches a fall. Once a second it prints
   `walk: command [vx, vy, yaw]  tilt ... rad`.
7. **Drive with the sticks.** Push them past about 60%: the policy stands still
   for small commands (see [Known limits](#known-limits)).
8. **Stop:** **L2 + B** (every motor damping only; the robot sags into the
   gantry), then Ctrl+C.

To try it without walking, start with `--zero-command`: the sticks are ignored
and the command stays (0, 0, 0), so the policy only balances in place.

## Remote controls

Hold the first button, then press the second.

| Buttons | Works from | What happens |
|---|---|---|
| L2 + Up | the start, Passive | **Stand:** the joints ramp from where they are to the default pose over 3 s, then hold |
| R1 + X | Stand, once the ramp is done and both sticks are centred | **Walk:** the policy runs at 50 Hz |
| L2 + B | anywhere | **Passive:** every motor gets no stiffness and damping kd 3 |

| Stick | Command | Full stick |
|---|---|---|
| Left, forward and back | walk forward and back | +0.5 and -0.5 m/s |
| Left, sideways | step left and right | 0.3 m/s |
| Right, sideways | turn left and right | 0.2 rad/s |

The axes are g1_ctrl's. A full stick gives the limit in `config.yaml`.

## What it does by itself

It refuses to start if:
- anything else publishes `rt/lowcmd` (it listens for 1 s first): two
  programs commanding the motors fight;
- no `rt/lowstate` arrives within 5 s;
- the robot is not the 29-DoF G1 (`mode_machine` 5);
- `config.yaml` has a missing or unknown key, or command limits beyond the
  policy's trained ranges.

Once running, it goes to **Passive** if:
- `rt/lowstate` stops for 0.1 s;
- the pelvis tilts past 1 rad while walking;
- a policy step fails (for example a state value that is not finite).

Ctrl+C, or any error in the script, sends Passive for 1 s and then exits.
After that the motors get no more commands from the laptop.

## Troubleshooting

| It prints | Why | What to do |
|---|---|---|
| `Something else is publishing rt/lowcmd` | Unitree's controller still has the robot | Put it in debug mode (step 2) and start again. If it persists, list the writers with `cyclonedds ls` (see [`../docs/bringup.md`](../docs/bringup.md)) |
| `No rt/lowstate within 5 s` | no link to the robot | Check the cable and `ip -br addr show dev enp130s0` |
| `R1 + X ignored: still moving to the stand pose` | pressed before the 3 s ramp ended | Wait, press again |
| `R1 + X ignored: centre both sticks first` | a stick was off centre | Let go of the sticks, press again |
| `walk: command [0.0, 0.0, 0.0]` while pushing a stick | started with `--zero-command` | Restart without it |
| The command shows, the robot does not step | command too small, or the gantry carries the weight | Push past 60%; slacken the gantry |

## Config

[`config.yaml`](config.yaml) is shared with `../sim2sim_rl/sim2sim.py`, so what
runs in the sim is what runs on the robot. Every key is required and an
unknown key is an error.

| Key | Now | Meaning |
|---|---|---|
| `policy_dir` | `../sim2sim_rl/policy/velocity_v0` | the policy, relative to the file |
| `commands.vx`, `vy`, `yaw` | [-0.5, 0.5], [-0.3, 0.3], [-0.2, 0.2] | command limits (m/s, m/s, rad/s). Must include 0 and stay inside the trained ranges in `deploy.yaml`: vx -0.5 to 1.0, vy -0.3 to 0.3, yaw -0.2 to 0.2 |
| `fall_tilt` | 1.0 | rad from upright before Passive (g1_ctrl's) |
| `passive_kd` | 3.0 | Passive damping (g1_ctrl's) |
| `robot.interface`, `domain_id` | `enp130s0`, 0 | the laptop's wired port and the robot's DDS domain |
| `robot.control_dt` | 0.002 | s between `rt/lowcmd` writes (500 Hz) |
| `robot.stand_seconds` | 3.0 | the stand ramp |
| `robot.state_timeout` | 0.1 | s without `rt/lowstate` before Passive |
| `robot.stick_deadzone` | 0.05 | stick values below this read as 0 |
| `robot.exit_passive_seconds` | 1.0 | Passive sent on Ctrl+C before exiting |

`--net` and `--domain` override the port and domain, and `--config` picks
another file.

## Known limits

From MuJoCo (`../sim2sim_rl/README.md` has the numbers):

- **It stands still for small commands:** forward below about 0.27 m/s and
  sideways below 0.3 m/s.
- **It barely turns on the spot.** It turns while walking forward (0.14 rad/s
  for 0.2). Training only asked it to turn within ±0.1 rad/s.
- **It was trained without hands;** ours carry the Dex3-1 hands, which this
  script does not command.

## How it works

- A 500 Hz loop reads the newest `rt/lowstate` and writes `rt/lowcmd`: all 29
  motors in position mode, with a target angle, stiffness kp and damping kd.
  The motors run their own PD on those.
- **Stand** sends a straight-line ramp to the default pose with the policy's
  gains: hips 100, knees 150, ankles 40, waist 200, arms 40.
- **Walk** runs the policy every 20 ms, with `../sim2sim_rl/g1_velocity_policy.py`:
  - the inputs are the pelvis IMU's orientation and angular velocity, the 29
    joint angles and velocities, the command and its last output, 5 steps of
    each;
  - each output is turned into a joint target, `default pose + 0.25 × output`;
  - the same targets are re-sent between policy steps.
- The remote is read from `rt/lowstate`'s `wireless_remote` bytes, in the
  layout of the SDK's `example/wireless_controller/wireless_controller.py`.

## Files

- `deploy_real.py`: the script.
- `config.yaml`: the settings shared with `../sim2sim_rl`.

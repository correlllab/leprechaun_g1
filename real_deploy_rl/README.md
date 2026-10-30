# real_deploy_rl

Runs the [`../sim2sim_rl`](../sim2sim_rl) walking policy on the real G1 from the
laptop over DDS, like Unitree's `g1_ctrl`. Ran on our G1 on 2026-10-30.

Setup: the network and the `g1` env in [`../docs/bringup.md`](../docs/bringup.md),
plus `pip install onnxruntime pyyaml`.

## Run

1. Robot on the gantry. On the remote, **L2 + R2** for debug mode.
2. Start it:
   ```bash
   conda activate g1
   systemd-inhibit --what=sleep:idle:handle-lid-switch --who=G1 --why="G1 RL" python deploy_real.py
   ```
3. **L2 + Up**: the stand pose over 3 s. Lower the feet to the ground.
4. **R1 + X** (sticks centred): the policy starts. Drive with the sticks.
5. **L2 + B**: damping. Then Ctrl+C.

`--zero-command` ignores the sticks, so it only balances in place.

## Controls

| Input | Action |
|---|---|
| L2 + Up | stand |
| R1 + X | walk |
| L2 + B | damping, any time |
| left stick | forward/back ±0.5 m/s, sideways ±0.3 m/s |
| right stick | turn ±0.2 rad/s |

Push the sticks past about 60%: the policy stands still for small commands.

## Safety

It refuses to start if another program publishes `rt/lowcmd`, and goes to
damping if `rt/lowstate` stops for 0.1 s or the robot tilts past 1 rad.

## Config

[`config.yaml`](config.yaml), shared with the sim: command limits (refused if
beyond the trained ranges), timings and the network port.

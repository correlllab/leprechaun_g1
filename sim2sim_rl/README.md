# sim2sim_rl

unitree_rl_lab's G1 walking policy in MuJoCo, run by a Python port of Unitree's
`g1_ctrl`. The same code drives the real robot in
[`../real_deploy_rl`](../real_deploy_rl).

[Video](sim2sim_walk.mp4): 0.5 m/s forward, turning 0.2 rad/s.

## Run

```bash
conda activate g1
python sim2sim.py --cmd 0.5 0 0                          # viewer; --cmd VX VY YAW
python sim2sim.py --headless --seconds 10 --cmd 0.5 0 0  # prints speeds and torques
python sim2sim.py --seconds 12 --cmd 0.5 0 0.2 --record walk.mp4
```

Commands are clipped to the limits in
[`../real_deploy_rl/config.yaml`](../real_deploy_rl/config.yaml), the file the
robot uses. `--hands` adds the Dex3-1 hands.

## Results

| Command | Result |
|---|---|
| forward 0.5 m/s | 0.49 m/s |
| back 0.5 m/s | 0.42 m/s |
| sideways 0.3 m/s | 0.25 m/s |
| forward 0.5 m/s + turn 0.2 rad/s | turns 0.14 rad/s |
| forward up to 0.2, sideways up to 0.25 m/s | stands still |
| turn on the spot | barely turns |

## Files

- `g1_velocity_policy.py`: observation and action (ported from `g1_ctrl`), and the config loader.
- `sim2sim.py`: the MuJoCo loop.
- `policy/velocity_v0/`: the policy, from unitree_rl_lab at 4960b84 (Apache 2.0, `policy/LICENSE.unitree_rl_lab`).

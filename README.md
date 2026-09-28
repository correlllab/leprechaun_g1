# leprechaun_g1

Correll Lab's Unitree G1 humanoid platform: hardware reference for our G1 and
Dex3-1 dexterous hands. University of Notre Dame.

## Layout

```
assets/     URDF and MuJoCo models for the G1 29 DoF and the Dex3-1 hands
docs/       bring-up notes and our unit's details
tools/      read the robot's state
```

## Quick start

See [assets/README.md](assets/README.md) for the Dex3-1 joint order and
mirroring, and the two loader gotchas worth knowing before you start.

To connect to the robot and read its state, see [docs/bringup.md](docs/bringup.md).

## Open hardware questions

These are unknowns about our specific unit, not about the models. They gate
anything torque-enabled:

- Ankle and waist roll/pitch torque limits. Unitree's URDF says 35 Nm, its MJCF
  50 Nm; see [assets/README.md](assets/README.md).
- Whether the Dex3-1 `thumb_0` joint mirrors between hands. The other six joints
  demonstrably do; `thumb_0` has a symmetric range, so the model cannot say.
- Quaternion convention (w-first or w-last) in whatever observation builder we
  deploy. A wrong sense of "down" is painful to debug from a fall video.

## License

Our code and documentation are MIT, see [LICENSE](LICENSE). The vendored robot
descriptions under `assets/` are Unitree's, BSD 3-Clause, see
[assets/LICENSE.unitree_ros](assets/LICENSE.unitree_ros).

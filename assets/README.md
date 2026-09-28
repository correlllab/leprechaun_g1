# Robot descriptions

URDF and MuJoCo (MJCF) models for our Unitree G1 and its Dex3-1 hands, vendored
from Unitree so the repo is self-contained and pinned. These are Unitree's
published models, not measurements of our unit, so every number in them still
has to be checked against the real robot during bring-up.

| Package | Contents |
| --- | --- |
| [`g1_description/`](g1_description/) | G1 29 DoF, with and without hands, URDF and MJCF, 51 meshes |
| [`dex3_1_description/`](dex3_1_description/) | Dex3-1 left and right hand, URDF plus generated MJCF, 16 meshes |

## Provenance

| | |
| --- | --- |
| Upstream | [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) |
| Commit | `ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee` (2026-09-16) |
| Paths | `robots/g1_description/`, `robots/dexterous_hand_description/dex3_1/` |
| License | BSD 3-Clause, see [`LICENSE.unitree_ros`](LICENSE.unitree_ros) |

Files are byte-for-byte upstream except `dex3_1_description/dex3_1_{l,r}.xml`
and `dex3_1_description/scene.xml`, which are ours (see [Dex3-1](#dex3-1)).
Keeping the rest unmodified means updating is a plain diff against upstream.

## Our robot: G1 29 DoF

We carry exactly the two variants for our robot. Everything else upstream ships
(23 DoF, dual-arm, lock-waist, Inspire hand, Dex1, `g1_comp`) is deliberately
not vendored.

| File | Use | DoF | `mode_machine` |
| --- | --- | :---: | :---: |
| `g1_29dof_rev_1_0.{urdf,xml}` | body only | 29 (12 leg, 3 waist, 14 arm) | 5 |
| `g1_29dof_with_hand_rev_1_0.{urdf,xml}` | body with both Dex3-1 hands | 43 (29 + 7*2) | 5 |

Hip pitch and roll gear ratios are {14.3, 22.5}; wrist motor is the 4010.

### Confirmed on our unit

Our G1 reports `mode_machine` 5 in `rt/lowstate` (read 2026-09-28), so these are
the right files. Unit details are in [docs/hardware/g1.md](../docs/hardware/g1.md).

### Torque limits disagree between URDF and MJCF

At the vendored commit the URDFs give the ankles and waist roll/pitch 35 Nm and
30 rad/s (changed upstream in `5ae4fa5`, 2025-03-27), while both MJCFs still say
50 Nm. Hip roll agrees at 139 Nm. Neither is a measurement; confirm on hardware
before either goes into a safety config.

## Dex3-1

Seven joints per hand, three fingers. The order below is the order Unitree's
`HandCmd_` motor array uses, which is also the order of the generated MJCF
actuators.

| Index | Joint |
| :---: | --- |
| 0 | `{side}_hand_thumb_0_joint` |
| 1 | `{side}_hand_thumb_1_joint` |
| 2 | `{side}_hand_thumb_2_joint` |
| 3 | `{side}_hand_middle_0_joint` |
| 4 | `{side}_hand_middle_1_joint` |
| 5 | `{side}_hand_index_0_joint` |
| 6 | `{side}_hand_index_1_joint` |

### Mirroring between hands

Left and right close along opposite joint signs, which the joint limits show
directly. Six of the seven joints have limits that are exact negations between
hands. `thumb_0` is the exception: its range is symmetric on both hands, so the
model cannot tell us whether it mirrors. That one has to be determined on
hardware.

| Joint | Left range | Right range | Mirrored |
| --- | --- | --- | --- |
| `thumb_0` | -1.0472, 1.0472 | -1.0472, 1.0472 | undetermined, symmetric range |
| `thumb_1` | -0.724312, 1.0472 | -1.0472, 0.724312 | yes |
| `thumb_2` | 0, 1.74533 | -1.74533, 0 | yes |
| `middle_0` | -1.5708, 0 | 0, 1.5708 | yes |
| `middle_1` | -1.74533, 0 | 0, 1.74533 | yes |
| `index_0` | -1.5708, 0 | 0, 1.5708 | yes |
| `index_1` | -1.74533, 0 | 0, 1.74533 | yes |

Ranges above are from `g1_29dof_with_hand_rev_1_0.xml`. Do not copy them into a
safety config. They are the model's limits, not measured ones, and `thumb_1`
disagrees between the standalone hand URDF (-0.610865) and the G1 model
(-0.724312).

### Generated MJCF

Unitree ships no standalone MJCF for the Dex3-1; the only MuJoCo version of the
hand is welded to the wrist inside `g1_29dof_with_hand_rev_1_0.xml`. So
`dex3_1_{l,r}.xml` are generated from the vendored URDFs:

```bash
python tools/make_dex3_mjcf.py
```

The palm is the root body, fixed to the world, so the hand can be attached to a
wrist or given a `<freejoint/>`. Actuators are plain force motors with the URDF
effort limits. The hand's real PD gains are a hardware parameter and are
deliberately not modelled.

## Using these files

```bash
python tools/view.py --list                                          # what is vendored
python tools/view.py assets/g1_description/g1_29dof_with_hand_rev_1_0.xml
python tools/view.py assets/dex3_1_description/scene.xml
python tools/check_assets.py                                         # everything parses
```

Two things that will bite otherwise:

**MuJoCo cannot load the URDFs directly.** Each URDF carries
`<mujoco><compiler meshdir="meshes"/></mujoco>` while its own mesh filenames
already begin with `meshes/`, so MuJoCo looks under `meshes/meshes/` and fails.
This is upstream's quirk and we keep it so the files stay unmodified. ROS and
Pinocchio load them as-is; `tools/view.py` works around it with a temporary copy.
Use the `.xml` files with MuJoCo.

**The G1 MJCFs already contain a scene.** Both `g1_*.xml` files define their own
floor, light and skybox, so they open in the viewer as-is and there is no
separate G1 scene file. The generated Dex3-1 models do not, which is why
`dex3_1_description/scene.xml` exists.

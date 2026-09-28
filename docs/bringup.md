# Bring-up: connect to the G1 and read its state

What worked on 2026-09-28 from an Ubuntu 24.04 laptop. Our unit's details are in
[hardware/g1.md](hardware/g1.md).

## Network

```
laptop ─┬─ enp130s0   192.168.123.222 ── cable ── robot switch ─┬─ 192.168.123.161  motion computer (Unitree's)
        │                                                        └─ 192.168.123.164  dev computer (ours)
        └─ WiFi ── internet
```

- The robot's network is a private wired subnet, `192.168.123.0/24`, with fixed
  addresses and no DHCP. Give the laptop's wired port a manual address,
  `192.168.123.222/24`, with no gateway, so internet traffic stays on WiFi.
- Use the cable. The SDK finds the robot by DDS multicast on the wired subnet,
  which does not reach across from WiFi.
- If pings come and go, check `ip -br link show enp130s0`; it should say UP.
  Ours flapped at 100 Mb/s until the cable connection was fixed, then linked at
  1 Gb/s.

## DDS

The robot talks DDS (CycloneDDS 0.10.2) on domain 0.
`unitree_sdk2_python` speaks DDS directly.

ROS 2, through Unitree's separate `unitree_ros2` repo, sees the same topics with `/` in place of `rt/`.

## Python environment

cyclonedds 0.10.2 ships wheels only up to Python 3.10:

```bash
conda create -n g1 python=3.10
conda activate g1
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python && pip install -e .
```


## List the topics

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp130s0"/></Interfaces></General></Domain></CycloneDDS>'
cyclonedds ls -r 5s --suppress-progress-bar --color none | grep -o 'rt/[A-Za-z0-9_/]*' | sort -u
```

`cyclonedds` comes with the cyclonedds package; the export points it at the
robot's port. The ones that matter first:

| Kind | Topics |
| --- | --- |
| State | `rt/lowstate` (joints, IMU, remote; `rt/lf/lowstate` is a lower-rate copy), `rt/dex3/{left,right}/state`, `rt/secondary_imu`, `rt/lf/bmsstate` (battery), `rt/sportmodestate`, `rt/odommodestate`, `rt/state_estimator/*` |
| Commands | `rt/lowcmd` (every motor), `rt/arm_sdk` (arms, while Unitree's controller balances), `rt/dex3/{left,right}/cmd` |
| Services | `rt/api/<name>/request` and `/response`, e.g. `sport`, `motion_switcher`, `arm`, `voice` |

## Read rt/lowstate

```bash
python tools/read_lowstate.py
```

Read-only: it subscribes and never publishes. Once a second it prints
`mode_machine`, the IMU roll, pitch and yaw, and each of the 29 joints by name
with its angle. Stop it with Ctrl+C.

`rt/lowstate` holds a small header, then the IMU, then the motors, then the remote:

```
LowState_
├─ version        [2]   firmware/version numbers
├─ mode_pr              0 = ankles and waist as pitch/roll, 1 = raw motor pairs A/B
├─ mode_machine         hardware variant ID
├─ tick                 counter, keeps increasing while data is live
├─ imu_state
│   ├─ quaternion     [4]  orientation
│   ├─ gyroscope      [3]  angular velocity (rad/s)
│   ├─ accelerometer  [3]  linear acceleration
│   ├─ rpy            [3]  roll, pitch, yaw (rad)
│   └─ temperature
├─ motor_state   [35]   one per motor slot; the G1 uses 0-28
│   ├─ q               joint angle (rad)
│   ├─ dq              joint velocity (rad/s)
│   ├─ ddq             joint acceleration
│   ├─ tau_est         estimated torque (N·m)
│   ├─ temperature [2] two temperature readings
│   ├─ vol             voltage
│   └─ mode, motorstate, sensor [2]   motor mode and status bits
├─ wireless_remote [40] the remote's buttons and sticks, as raw bytes
└─ reserve, crc          spare fields and a checksum
```

Motor slots: left leg 0-5, right leg 6-11, waist 12-14 (yaw, roll, pitch), left
arm 15-21, right arm 22-28. `dq` is derived from the joint encoder and `tau_est`
from motor current; the joints have no velocity or torque sensors. The hands'
state topics carry the same motor fields, 7 motors per hand.

## mode_machine

`mode_machine` names the hardware variant. Ours is 5.

| `mode_machine` | Waist joints |
| --- | --- |
| 5 | 3; matches `assets/g1_description/` |

The full table is in unitree_ros, `robots/g1_description/README.md`.

# Ship Emulation

Uses a UR16 robot arm to physically emulate ship motion for hardware-in-the-loop testing. The UR16 tracks real-time pose data from an external ship simulation, allowing another robot (the device under test) to respond to realistic ship motion as if it were mounted on an actual vessel.

## Concept

A ship simulation provides timestamped pose data in XYZ RPY format. This subproject reads that data stream and replays it on the UR16 via `common_robot_interface`, effectively turning the robot arm into a 6-DOF motion platform. The robot under test is mounted on (or interacts with) the UR16 end-effector and experiences the emulated ship motion.

```
Ship simulation ──(XYZ RPY + timestamp)──> ship_emulation ──(RTDE)──> UR16 arm
                                                                            │
                                                                     Robot under test
```

## Input Data Format

The simulation provides pose data as:

```
timestamp, x, y, z, roll, pitch, yaw
```

- `timestamp`: seconds (float), used to preserve the original motion timing
- `x, y, z`: position in mm, relative to the robot base frame
- `roll, pitch, yaw`: orientation in degrees (RPY / Euler XYZ convention)

## Dependencies

- `common_robot_interface` (included in this repo) — UR16 is controlled via the RTDE interface (`RTDEController`)
- Python 3.8+
- See `requirements.txt` in the repo root

## Usage

_To be filled in as scripts are added._

## Notes

- The UR16 workspace limits the range of achievable ship motion — verify that the simulation trajectory stays within the robot's reachable workspace before running.
- Set the work frame in the controller to match your physical setup (base frame origin relative to the test rig).
- Linear and angular speed limits on the robot may mean fast ship transients are not perfectly reproduced; tune `linear_speed` and `angular_speed` accordingly.

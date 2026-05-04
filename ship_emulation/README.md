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

- `timestamp`: seconds (float), monotonically increasing within a session
- `x, y, z`: position in mm, relative to the configured work frame
- `roll, pitch, yaw`: orientation in degrees — RPY convention (roll=X, pitch=Y, yaw=Z), which maps directly to CRI's default `sxyz` Euler axes

## Dependencies

- `common_robot_interface` (included in this repo) — UR16 is controlled via `RTDEController` → `SyncRobot` → `AsyncRobot`
- `numpy` — already in repo `requirements.txt`
- Python 3.8+

## Planned File Structure

```
ship_emulation/
├── __init__.py          ✅ created (empty package marker)
├── README.md            ✅ this file
├── config.py            ⬜ all configuration dataclasses
├── data_source.py       ⬜ ShipPose dataclass + DataSource ABC + CsvFileSource + UdpSource
├── safety.py            ⬜ SafetyChecker (workspace bounds, rate-of-change, timestamp check)
├── robot_interface.py   ⬜ UR16Interface wrapping AsyncRobot/RTDEController
├── emulator.py          ⬜ ShipEmulator main run-loop
└── run.py               ⬜ CLI entry point
```

---

## Implementation Status

**Branch:** `ship-emulation`  
**Started:** architecture designed and reviewed, `__init__.py` created, writing interrupted before module files were written.

### Architecture decisions (agreed, ready to implement)

#### `config.py`
Three dataclasses:
- `RobotConfig` — IP, TCP offset, work frame, home joint angles, linear/angular speed & accel, blend radius
- `WorkspaceLimits` — per-axis min/max for x/y/z (mm) and roll/pitch/yaw (deg)
- `SafetyConfig` — wraps `WorkspaceLimits` + `max_linear_rate` (mm/s), `max_angular_rate` (deg/s), `min_dt` (s)
- `EmulatorConfig` — wraps `RobotConfig` + `SafetyConfig`

#### `data_source.py`
- `ShipPose` — frozen dataclass: `timestamp, x, y, z, roll, pitch, yaw`; has `as_robot_pose() -> tuple` returning `(x, y, z, roll, pitch, yaw)` directly compatible with CRI `sxyz` axes
- `DataSource` — ABC with `poses() -> Iterator[ShipPose]` and `close()`; supports context manager
- `CsvFileSource` — reads `timestamp,x,y,z,roll,pitch,yaw` CSV; optional header skip; validates column count and types
- `UdpSource` — receives JSON datagrams `{"timestamp":…, "x":…, …}`; configurable host/port/timeout/max_poses

#### `safety.py`
- `SafetyViolation(Exception)` — raised on any check failure; catching this must always result in robot stop
- `SafetyChecker` — stateful (tracks previous pose for rate checks); `check(pose)` raises or returns:
  1. Workspace bounds (per-axis, all 6 DOF)
  2. Linear rate of change: `‖Δxyz‖ / Δt ≤ max_linear_rate`
  3. Angular rate of change: `max(|Δangle|) / Δt ≤ max_angular_rate` (worst-axis, conservative)
  4. Timestamp monotonicity and minimum step size
- `reset()` to clear state between sessions

#### `robot_interface.py`
- `UR16Interface` — context manager (`__enter__` calls `connect()`, `__exit__` calls `close()`)
- Internally holds `AsyncRobot(SyncRobot(RTDEController(...)))`
- `connect()` — opens RTDE, sets tcp/work_frame/speeds/accel/blend_radius from config
- `move_home()` — joint-space move to `home_joint_angles` (predictable path, safe from any start pose); blocks
- `move_to(pose)` — `async_move_linear`; waits for any in-flight move first; returns immediately
- `wait_for_motion()` — blocks until current async move done
- `is_motion_done()` — non-blocking check
- `emergency_stop()` — calls `controller.stop_linear_velocity(5000 mm/s²)` then closes connection; documented as software-only backstop — physical E-stop is always primary
- `current_pose` / `current_joint_angles` properties for diagnostics
- Internal `_motion_pending` flag guards against calling `async_result()` when idle

#### `emulator.py`
- `ShipEmulator` — takes `DataSource`, `UR16Interface`, `SafetyChecker`, `EmulatorConfig`; does NOT own lifecycle of these (caller manages context managers)
- `run()`:
  1. Resets safety checker
  2. Registers `SIGINT`/`SIGTERM` handlers → graceful stop flag
  3. Calls `move_home()`
  4. Iterates `source.poses()` — first pose sets wall-clock + sim-time origin
  5. For each pose: `safety.check()` → sleep to sim timestamp → `interface.move_to()`
  6. If robot still moving when next pose is due: logs warning, waits (`is_motion_done` check + warning with advice to reduce speed or lower simulation rate)
  7. On `SafetyViolation`: `emergency_stop()`, raises `EmulationError`
  8. On graceful stop / source exhausted: `wait_for_motion()` then `move_home()`
- Timing: `time.monotonic()` based; sleeps are skipped if `< 5 ms` (avoid OS sleep overhead noise)
- Logs pose count and late-move count on completion

#### `run.py`
- `argparse` CLI: `--source {csv,udp}`, `--csv-file`, `--udp-port`, `--robot-ip`, `--x/y/z-range`, `--dry-run`, `--log-level`
- `--dry-run`: iterates source + safety checker only, no robot connection
- Live run: prints safety warning, requires `ENTER` keypress before connecting robot
- Exit codes: 0 = success, 1 = runtime error, 2 = bad arguments

### Key CRI API notes (from reading the source)
- `RTDEController` exposes `linear_accel` and `angular_accel` directly (not via `SyncRobot`); set on the controller before wrapping
- `AsyncRobot.async_move_linear(pose)` → non-blocking; `async_result()` → blocks; `async_done()` → non-blocking check
- Raises `AsyncBusy` if `async_move_linear` called while already busy — `UR16Interface` guards this with `_motion_pending`
- `stop_linear_velocity(accel_mm_s2)` is on `RTDEController` directly (not on `SyncRobot`/`AsyncRobot`); access via `interface._controller`
- Euler convention: CRI default is `sxyz` (static/extrinsic XYZ) = intrinsic ZYX = standard RPY; pass `(roll, pitch, yaw)` as `(alpha, beta, gamma)` — no conversion needed

## Usage (once implemented)

```bash
# Replay a CSV file
python -m ship_emulation.run --source csv --csv-file trajectory.csv --robot-ip 192.168.1.100

# Receive live UDP data from simulation
python -m ship_emulation.run --source udp --udp-port 5005 --robot-ip 192.168.1.100

# Validate data without touching the robot
python -m ship_emulation.run --source csv --csv-file trajectory.csv --dry-run
```

## Notes

- The UR16 workspace limits the range of achievable ship motion — verify that the simulation trajectory stays within the robot's reachable workspace before running. Use `--x-range`, `--y-range`, `--z-range` CLI flags to set safe bounds.
- Set `work_frame` in `RobotConfig` to match your physical setup (pose of the emulation origin in the robot base frame).
- If the robot cannot keep up with the simulation rate (logged as "late" moves), reduce `max_linear_rate`/`max_angular_rate` in `SafetyConfig`, or increase `linear_speed`/`angular_speed` in `RobotConfig` — within safe limits.
- **Physical E-stop must always be within operator reach when the robot is powered.**

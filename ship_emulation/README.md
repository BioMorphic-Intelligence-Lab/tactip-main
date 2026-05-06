# Ship Emulation

Uses a UR16 robot arm to physically emulate ship motion for hardware-in-the-loop testing. The UR16 replays pre-recorded vessel motion data, allowing another robot (the device under test) to respond to realistic ship motion as if it were mounted on an actual vessel.

## Concept

A ship simulation provides timestamped pose data in XYZ RPY format. This subproject reads that data, applies optional augmentation and smoothing, and replays it on the UR16 via `common_robot_interface`, effectively turning the robot arm into a 6-DOF motion platform.

```
OrcaFlex simulation ──(CSV export)──> ship_emulation pipeline ──(RTDE)──> UR16 arm
                                                                               │
                                                                        Robot under test
```

## Application Context

This subproject is part of a **Science Robotics** paper on aerial tactile servoing. The contribution is a local-contact-frame formulation for aerial manipulator control; ship motion emulation is the demonstration task because it excites all 6 DOF in a physically motivated way.

Vessel motion data is generated in **OrcaFlex 11** (103 m LOA vessel). The pipeline handles the gap between raw simulation output and robot-safe replay.

### Dataset status

| Channel | Raw data | Target | Status |
|---|---|---|---|
| Linear z (largest) | ±1.1 m, 1.1 m/s peak | ±0.6 m, ≤0.3 m/s | Pending sim change + smoothing |
| Angular (all axes) | ±1.9°, 1.6 °/s peak | ±15°, proportional | Pending sim change |

**Planned OrcaFlex changes (to request from collaborator):**
1. Move measurement point to vessel CoG — eliminates lever-arm amplification of linear channels
2. Tune sea state peak frequency toward natural roll period, or reduce vessel GM — drives angular amplitude to ±15°

After those changes, gentle spline smoothing on the linear channels is still expected to be necessary to trim peak velocity (physical constraint: v_max = 2πf·A at ~0.1 Hz gives ~0.38 m/s for ±0.6 m amplitude). This is reported in the paper as a robot safety measure, not a physics intervention.

**Post-processing approach chosen:**
- Spline smoothing (`SmoothedSource`) on linear channels for velocity management
- No spectral scaling or sinusoidal overlay planned — keeps OrcaFlex data as the sole physics source

---

## Input Data Format

```
time, linear_x, linear_y, linear_z, rotation_x, rotation_y, rotation_z
```

- `time`: seconds (float), monotonically increasing
- `linear_x/y/z`: position in **metres**, relative to the configured work frame
- `rotation_x/y/z`: orientation in **degrees** — RPY convention (roll=X, pitch=Y, yaw=Z), maps directly to CRI's default `sxyz` Euler axes

`AugmentedSource` applies `linear_scale` (default 1000, m → mm) before poses reach the robot.

---

## Pipeline

```
CsvFileSource
    └─> AugmentedSource      (unit conversion, optional angular scale, optional sinusoidal overlay)
            └─> SmoothedSource (optional; spline smoothing per channel group)
                    └─> ShipEmulator / SafetyChecker → UR16Interface
```

---

## Dependencies

- `common_robot_interface` (included in this repo) — UR16 controlled via `RTDEController` → `SyncRobot` → `AsyncRobot`
- `numpy`, `scipy` — in repo `requirements.txt`
- `matplotlib` — for `analyze_data.py` only
- Python 3.8+

---

## File Structure

```
ship_emulation/
├── __init__.py          ✅ empty package marker
├── README.md            ✅ this file
├── analyze_data.py      ✅ data inspection, augmentation preview, smoothing preview
├── config.py            ✅ all configuration dataclasses
├── data_source.py       ✅ ShipPose + DataSource ABC + CsvFileSource + AugmentedSource + SmoothedSource
├── safety.py            ✅ SafetyChecker (workspace bounds, rate-of-change, timestamp check)
├── robot_interface.py   ✅ UR16Interface wrapping AsyncRobot/RTDEController
├── emulator.py          ✅ ShipEmulator main run-loop
├── run.py               ✅ CLI entry point
└── vessel_motion_clean.csv  (current working dataset — OrcaFlex export, pre-CoG/sea-state update)
```

---

## Implementation Status

**Branch:** `ship-emulation`  
**Status:** all modules implemented and functional; dataset update pending (see above).

### Architecture

#### `config.py`
Five dataclasses:
- `RobotConfig` — IP, TCP offset, work frame, home joint angles, linear/angular speed & accel, blend radius
- `WorkspaceLimits` — per-axis min/max for x/y/z (mm) and roll/pitch/yaw (deg)
- `SafetyConfig` — wraps `WorkspaceLimits` + `max_linear_rate` (mm/s), `max_angular_rate` (deg/s), `min_dt` (s)
- `SinusoidalOverlay` — per-axis amplitude (deg) and frequency (Hz) for roll, pitch, yaw
- `AugmentationConfig` — `linear_scale` (default 1000.0, m→mm), `angular_scale` (default 1.0), optional `SinusoidalOverlay`
- `EmulatorConfig` — wraps `RobotConfig` + `SafetyConfig`

#### `data_source.py`
- `ShipPose` — frozen dataclass: `timestamp, x, y, z, roll, pitch, yaw`; `as_robot_pose()` returns `(x, y, z, roll, pitch, yaw)` compatible with CRI `sxyz` axes
- `DataSource` — ABC with `poses() -> Iterator[ShipPose]` and `close()`; supports context manager
- `CsvFileSource` — reads CSV with header `time,linear_x,…,rotation_z`; validates column count and types per row
- `AugmentedSource` — wraps any `DataSource`; applies `AugmentationConfig` (linear/angular scaling + sinusoidal overlay) to each pose on the fly
- `SmoothedSource` — buffers all poses, fits `scipy.UnivariateSpline` per channel, re-emits poses at original timestamps with smoothed values. Separate smoothing factors for linear (`s_linear`, in mm²) and angular (`s_angular`, in deg²) channels — critical distinction because the same `s` value has very different effect at mm vs degree scale. `s=0` interpolates exactly; larger = smoother. Good starting range: `s_linear=1e4–1e9`, `s_angular=1e1–1e3`.

#### `safety.py`
- `SafetyViolation(Exception)` — raised on any check failure; must always result in robot stop
- `SafetyChecker` — stateful; `check(pose)` validates:
  1. Workspace bounds (per-axis, all 6 DOF)
  2. Linear rate: `‖Δxyz‖ / Δt ≤ max_linear_rate`
  3. Angular rate: `max(|Δangle|) / Δt ≤ max_angular_rate` (worst-axis)
  4. Timestamp monotonicity and minimum step size
- `reset()` clears state between sessions

#### `robot_interface.py`
- `UR16Interface` — context manager; `__enter__` calls `connect()`, `__exit__` calls `close()`
- `connect()` — opens RTDE, sets tcp/work_frame/speeds/accel/blend_radius from config
- `move_home()` — joint-space move to `home_joint_angles`; blocks until CRI acknowledges
- `wait_until_stopped()` — polls actual TCP velocity via RTDE until below threshold (default 2 mm/s); called after `move_home()` before starting playback to avoid C204A1 protective stops caused by commanding `moveL` while the robot is still decelerating from a joint move
- `move_to(pose)` — `async_move_linear`; waits for any in-flight move first; returns immediately
- `wait_for_motion()` — blocks until current async move done
- `is_motion_done()` — non-blocking check
- `emergency_stop()` — `controller.stop_linear_velocity(5000 mm/s²)` then closes; software backstop only — physical E-stop is always primary
- `current_pose` / `current_joint_angles` — diagnostic properties

#### `emulator.py`
- `ShipEmulator` — takes `DataSource`, `UR16Interface`, `SafetyChecker`, `EmulatorConfig`; does not own lifecycle of these
- `run()` sequence:
  1. Reset safety checker
  2. Register `SIGINT`/`SIGTERM` handlers (set flag **and** raise `KeyboardInterrupt` to unblock network calls)
  3. `move_home()` → `wait_until_stopped()`
  4. Consume first pose from source, safety-check it, blocking move to start position
  5. Timed loop: for each remaining pose → `safety.check()` → sleep to sim timestamp → `move_to()`
  6. If robot still moving when next pose is due: log warning, wait (advises reducing speed or rate)
  7. On `SafetyViolation`: `emergency_stop()`, raise `EmulationError`
  8. On graceful stop or source exhausted: `wait_for_motion()` → `move_home()`
- Timing: `time.monotonic()`; sleeps < 5 ms are skipped
- Logs total pose count and late-move count on completion

#### `run.py`
- CLI args: `--csv-file` (required), `--robot-ip`, `--x/y/z-range`, `--linear-scale`, `--angular-scale`, `--overlay-{roll,pitch,yaw}-{amp,freq}`, `--smooth-linear`, `--smooth-angular`, `--dry-run`, `--log-level`
- `--smooth-linear` / `--smooth-angular`: spline smoothing factors (0 = disabled); passed to `SmoothedSource` when either is non-zero
- `--dry-run`: iterates full source pipeline + safety checker, no robot connection
- Live run: prints safety warning, requires `ENTER` before connecting
- `__main__` block contains explicit arg values for the current working configuration — edit this to persist a specific setup

#### `analyze_data.py`
- Standalone inspection script; run with `python -m ship_emulation.analyze_data`
- Same CLI args as `run.py` for augmentation and smoothing, plus `--save OUT.png`
- Prints stats table (min/max/mean/std) for position channels — for both raw and smoothed data when smoothing is enabled
- Prints rate stats (linear rate ‖Δpos‖/Δt and angular rate max|Δrot|/Δt) for raw and smoothed data
- Plots: position time series, rotation time series, XY trajectory, XZ trajectory, linear rate, angular rate — raw faded + smoothed overlay when smoothing is enabled; p95 lines on rate plots for both raw and smoothed
- `__main__` block contains explicit args for quick interactive use

### Key CRI API notes
- `RTDEController.linear_accel` / `angular_accel` set on the controller before wrapping in `SyncRobot`
- `AsyncRobot.async_move_linear(pose)` non-blocking; `async_result()` blocks; `async_done()` non-blocking check
- `stop_linear_velocity(accel)` is on `RTDEController` directly
- CRI default Euler convention is `sxyz` (static/extrinsic XYZ = intrinsic ZYX = standard RPY) — pass `(roll, pitch, yaw)` directly, no conversion needed
- `_wait_for_command_complete()` in the CRI internals returns when URScript acknowledges the command, **not** when the robot physically stops — hence `wait_until_stopped()` is needed after joint moves

---

## Usage

```bash
# Always run as a module from the repo root
cd /path/to/ats-meta

# Inspect raw data
python -m ship_emulation.analyze_data --csv ship_emulation/vessel_motion_clean.csv

# Preview with m→mm scaling and smoothing (tune s values here before committing to robot)
python -m ship_emulation.analyze_data \
    --linear-scale 1000 \
    --smooth-linear 1e9 \
    --smooth-angular 1e2 \
    --save preview.png

# Dry run (validate full pipeline, no robot)
python -m ship_emulation.run \
    --csv-file ship_emulation/vessel_motion_clean.csv \
    --dry-run

# Live run with smoothing
python -m ship_emulation.run \
    --csv-file ship_emulation/vessel_motion_clean.csv \
    --robot-ip 172.17.0.2 \
    --smooth-linear 1e9

# Or just edit the __main__ block in run.py and run:
python -m ship_emulation.run
```

---

## Notes

- **`work_frame`** in `RobotConfig` must match your physical setup — it defines the Cartesian origin for CSV pose offsets. CSV data contains relative displacements, so if `work_frame=(0,0,0,0,0,0)` the robot will try to reach absolute coordinates from its base frame, which are unreachable.
- **`home_joint_angles`** in `RobotConfig` is a joint-space parking position unrelated to `work_frame`. It should correspond to a safe, reachable configuration within the working area.
- If the robot cannot keep up with the simulation rate (logged as "late" moves), reduce `SafetyConfig.max_linear_rate` / `max_angular_rate`, or increase `RobotConfig.linear_speed` / `angular_speed` — within safe limits. Alternatively, increase the smoothing factor to reduce peak velocity in the data.
- **Physical E-stop must always be within operator reach when the robot is powered.**

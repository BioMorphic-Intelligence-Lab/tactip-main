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

Ship motion emulation is the demonstration experiment for a novel controller because it excites all 6 DOF in a physically motivated way.

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
- `rclpy`, `geometry_msgs` — **optional**; required for ROS 2 pose logging via `RosBridge`. If not installed, `RosBridge` degrades silently to a no-op.
- Python 3.8+

---

## File Structure

```
ship_emulation/
├── __init__.py              ✅ empty package marker
├── README.md                ✅ this file
├── config.py                ✅ all configuration dataclasses
├── data_source.py           ✅ ShipPose, DataSource ABC, CsvFileSource, AugmentedSource,
│                               SmoothedSource, RigidBodyOffsetSource
├── safety.py                ✅ SafetyChecker (workspace bounds, rate-of-change, timestamp check)
├── robot_interface.py       ✅ UR16Interface wrapping AsyncRobot/RTDEController
├── emulator.py              ✅ ShipEmulator main run-loop (on_move callback hook)
├── motion_primitives.py     ✅ ChirpSource, TrapezoidalMoveSource, DwellSource,
│                               FadeInSource, SequentialSource
├── ros_bridge.py            ✅ RosBridge — shared ROS 2 publisher for commanded + actual poses
├── phase1.py                ✅ Phase 1 TacTip characterisation script
├── phase2.py                ✅ Phase 2 ship motion emulation script
├── run.py                   ✅ general-purpose CLI entry point
├── analyze_data.py          ✅ data inspection, augmentation preview, smoothing preview
├── 1_vessel_motion_clean.csv  (sea state 1 — default for Phase 2)
├── 2_vessel_motion_clean.csv  \
├── 2a_vessel_motion_clean.csv  > sea state 2 variants
├── 2b_vessel_motion_clean.csv /
└── 3_vessel_motion_clean.csv  (sea state 3)
```

All CSV files: 10 Hz sample rate, ~1 hour duration, columns: `time, x, y, z, roll, pitch, yaw` (positions in metres, angles in degrees).

---

## Experimental Campaign

The experiment is structured in three phases.

### Phase 0
Free-form testing of all Phase 1 and Phase 2 motions on the UR16 before the UAM establishes contact. No specific motion profiles required — use the phase scripts directly.

### Phase 1 — TacTip characterisation (`phase1.py`)

Systematic DoF-by-DoF characterisation of the TacTip sensor response using **trapezoidal velocity profiles** — no sinusoidal or chirp excitation.

**Loop structure — axes outer, velocity levels inner.** Each DoF is fully characterised across all velocity levels before moving to the next DoF. Within a DoF, the robot never returns home between velocity levels — it moves directly from the previous sweep endpoint to the next sweep start at approach speed.

**Experiment flow per run:**
1. ENTER → connect to robot
2. ENTER → move to home position
3. For each DoF (Rx, Ry, shear-x, shear-y, depth-z):
   - ENTER → start DoF characterisation (shows full velocity schedule)
   - For each velocity level (low → high):
     - ENTER → move slowly to sweep start position (`APPROACH_VELOCITY`)
     - ENTER → begin sweeps (`TRAP_N_CYCLES` one-way sweeps, `DWELL_SETTLE` pause after each)
     - ENTER → return to home  *(last level of each DoF only)*

**Motion shape:** sweeps alternate between `SWEEP_START` and `SWEEP_END` for each axis. Both values are configured explicitly — they do not have to be symmetric about zero. An axis whose `SWEEP_START` is non-zero requires a slow approach from home; `depth-z` defaults to `SWEEP_START = 0` so no approach is needed.

**Key configuration parameters** (edit at the top of `phase1.py`):

| Parameter | Default | Notes |
|---|---|---|
| `AXES` | `["roll","pitch","x","y","z"]` | DoF order; remove any entry to skip it |
| `SWEEP_START` | −45 deg / −400 mm (x,y) / 0 mm (z) | Sweep start position per axis |
| `SWEEP_END` | +45 deg / +400 mm (x,y) / +300 mm (z) | Sweep end position per axis |
| `TRAP_ACCELERATION` | 45 deg/s² / 100 mm/s² (x,y) / 50 mm/s² (z) | Ramp distance = v²/(2a) |
| `TRAP_VELOCITIES` | 3 levels, e.g. roll: 3→5→10 deg/s | List of dicts, one per level |
| `TRAP_N_CYCLES` | 5 | One-way sweeps per DoF per level |
| `APPROACH_VELOCITY` | 1 deg/s / 5 mm/s | Speed for slow positioning moves (approach and inter-level transitions) |
| `DWELL_SETTLE` | 5 s | Pause between consecutive sweeps |
| `BLEND_RADIUS` | 3 mm | Move pipelining; set to 0 for strict stop-and-go |
| `SAMPLE_RATE` | 10 Hz | Matches CRI/RTDE throughput limit |

Run:
```bash
python -m ship_emulation.phase1
```

### Phase 2 — Ship motion emulation (`phase2.py`)
Randomly samples a configurable window (default 3 minutes) from a vessel motion CSV file and replays it on the UR16. A raised-cosine fade-in (default 10 s) ramps the amplitude from zero at the start, preventing a velocity step when the UAM is already in contact.

**Key configuration parameters** (edit at the top of `phase2.py`):

| Parameter | Default | Notes |
|---|---|---|
| `CSV_FILE` | `1_vessel_motion_clean.csv` | Path to vessel motion data |
| `WINDOW_DURATION` | 180 s | Length of sampled window |
| `RANDOM_SEED` | `None` | Set to an int for reproducibility |
| `FADE_IN_ENABLED` | `True` | Smooth onset ramp |
| `FADE_IN_DURATION` | 10 s | Raised-cosine ramp duration |
| `LINEAR_SCALE` | 1000.0 | m → mm conversion |

The selected window timestamps are logged at INFO level for post-processing cross-reference.

Run:
```bash
python -m ship_emulation.phase2
```

---

## Data Collection

Both phase scripts publish to ROS 2 via `RosBridge` and are designed to run alongside a `ros2 bag record` session (or a launch-file bag recorder).

### Topics

| Topic | Type | Content |
|---|---|---|
| `/robot/cmd_pose` | `geometry_msgs/PoseStamped` | Commanded pose, published after each `move_to` |
| `/robot/actual_pose` | `geometry_msgs/PoseStamped` | Actual FK pose from RTDE, polled at 50 Hz |

Both topics publish in the **work frame**, with position in **metres** and orientation as a **quaternion** (converted from intrinsic-XYZ Euler degrees). The `frame_id` field encodes experiment context:
- Phase 1: `shear-x/level_2` — DoF outermost, then velocity level
- Phase 2: `phase2`

### Workflow

```bash
# Terminal 1 — bag recorder (or handled by launch file)
ros2 bag record /robot/cmd_pose /robot/actual_pose <mocap_topic> <other_topics>

# Terminal 2 — run the experiment
source venv-tactip/bin/activate
python -m ship_emulation.phase1   # or phase2
```

The UR robot driver topics (`/joint_states`, `/tf`, etc.) are already published by the driver and captured by the bag recorder independently.

### Adding RosBridge to future scripts

```python
from ship_emulation.ros_bridge import RosBridge

bridge = RosBridge("my_experiment")
bridge.start_feedback(lambda: interface.current_pose, rate_hz=50.0)
bridge.set_context("some/label")          # optional; updates frame_id
emulator = ShipEmulator(..., on_move=bridge)
emulator.run()
bridge.close()
```

`RosBridge` is a no-op if `rclpy` is not importable — scripts run identically with or without ROS 2 available.

---

## Implementation Status

**Branch:** `ship-emulation`  
**Status:** all modules implemented; Phase 1 and Phase 2 scripts ready for hardware trials. Dataset update pending (see above).

### Architecture

#### `config.py`
Five dataclasses:
- `RobotConfig` — IP, TCP offset, work frame, `home_pose` (work-frame Cartesian, default `(0,0,0,0,0,0)`), linear/angular speed & accel, blend radius
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
- `RigidBodyOffsetSource` — translates CoG motion to a fixed body-frame offset point using `R(roll,pitch,yaw) @ offset`

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
- `move_home()` — linear move to `RobotConfig.home_pose` in the work frame; synchronous/blocking. Because home is Cartesian, the emulator's automatic "move to first pose" step is a guaranteed no-op when sequences start from zero.
- `move_linear_at(pose, linear_speed, angular_speed)` — blocking linear move at explicitly specified speeds; temporarily overrides the robot's configured speeds and restores them afterwards. Used for slow approach and inter-axis positioning moves.
- `wait_until_stopped()` — polls actual TCP velocity via RTDE until below threshold (default 2 mm/s); call after any blocking move to confirm the robot has physically settled before issuing the next command
- `move_to(pose)` — `async_move_linear`; waits for any in-flight move first; returns immediately
- `wait_for_motion()` — blocks until current async move done
- `is_motion_done()` — non-blocking check
- `emergency_stop()` — `controller.stop_linear_velocity(5000 mm/s²)` then closes; software backstop only — physical E-stop is always primary
- `current_pose` / `current_joint_angles` — diagnostic properties; `current_pose` returns `(x, y, z, roll, pitch, yaw)` in mm/deg in the work frame

#### `emulator.py`
- `ShipEmulator(source, interface, safety, config, on_move=None, home_at_start=True, home_at_end=True)` — optional `on_move` callable receives each `ShipPose` immediately after `move_to`. `home_at_start/end` flags control whether `move_home()` is called at the start and end of `run()`; set both to `False` in `phase1.py` sweep runs so the robot stays in position between emulator calls.
- `run()` sequence:
  1. Reset safety checker
  2. Register `SIGINT`/`SIGTERM` handlers (set flag **and** raise `KeyboardInterrupt` to unblock network calls)
  3. If `home_at_start`: `move_home()` → `wait_until_stopped()`
  4. Consume first pose from source, safety-check it, blocking move to start position (no-op when robot is already there)
  5. Timed loop: for each remaining pose → `safety.check()` → sleep to sim timestamp → `move_to()` → `on_move(pose)`
  6. If robot still moving when next pose is due: log warning, wait (advises reducing speed or rate)
  7. On `SafetyViolation`: `emergency_stop()`, raise `EmulationError`
  8. On graceful stop or source exhausted: `wait_for_motion()` → if `home_at_end`: `move_home()`
- Returns `True` on normal completion, `False` if interrupted by Ctrl+C — callers raise `_Aborted` to unwind cleanly
- Timing: `time.monotonic()`; sleeps < 5 ms are skipped
- Logs total pose count and late-move count on completion

#### `motion_primitives.py`
Generates synthetic motion trajectories as `DataSource` implementations, all on a single axis with other axes held at zero. Used by `phase1.py`; composable for future sequences.

- `ChirpSource` — linear frequency sweep: `A·sin(φ(t))` where φ increases instantaneous frequency from `f_start` to `f_end` over `duration`. Peak velocity = 2π·f_end·A — verify against `SafetyConfig.max_linear_rate`. (Available but not used in current phase scripts.)
- `TrapezoidalMoveSource` — moves through ordered waypoints with trapezoidal (or triangular if distance is short) velocity profiles. Ramp distance = v_max²/(2·accel). Supports optional dwell hold at intermediate waypoints.
- `DwellSource` — holds a fixed position for a given duration; used as settling pauses between sweeps.
- `FadeInSource` — wraps any `DataSource`; scales deviations from the first pose by a raised-cosine ramp over `duration` seconds. Set `enabled=False` to bypass. Used in Phase 2 to avoid velocity step at contact.
- `SequentialSource` — concatenates multiple `DataSource` instances, stitching timestamps seamlessly. Logs a transition message at INFO level when each labelled source starts (set `source.name` to enable).

All primitives are composable and pass through the existing `ShipEmulator` + `SafetyChecker` stack unchanged.

#### `ros_bridge.py`
Shared ROS 2 publisher for commanded and actual robot poses. Designed to work with any `ShipEmulator`-based script without restructuring it as a ROS 2 node — rclpy spins in a daemon thread, leaving the main process as a plain Python script.

- `RosBridge(experiment)` — initialises rclpy, creates a node named `<experiment>_ros_bridge`, and starts the spin thread. `experiment` prefixes the `frame_id` on all published messages.
- `set_context(str)` — appends a sub-label to `frame_id` (e.g. `"level_2/shear-x"`); call before each axis or trial run to make poses filterable in the bag.
- `__call__(pose)` — publishes to `/robot/cmd_pose`; used as the `on_move` callback for `ShipEmulator`. Accepts `ShipPose` or a raw `(x, y, z, roll, pitch, yaw)` tuple.
- `start_feedback(get_pose, rate_hz=50)` — polls `get_pose()` in a daemon thread and publishes to `/robot/actual_pose`. Pass `lambda: interface.current_pose`.
- `stop_feedback()` / `close()` — stop the feedback thread and shut down rclpy cleanly.
- Orientation conversion: intrinsic-XYZ Euler degrees → quaternion, computed analytically (no extra dependencies beyond `math`).
- Silently becomes a no-op if `rclpy` is not importable.

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
- `_wait_for_command_complete()` in the CRI internals returns when URScript acknowledges the command, **not** when the robot physically stops — hence `wait_until_stopped()` is needed after both joint and linear blocking moves

---

## Usage

```bash
# Always run as a module from the repo root, with venv active
source venv-tactip/bin/activate

# Inspect raw data
python -m ship_emulation.analyze_data --csv ship_emulation/1_vessel_motion_clean.csv

# Preview with m→mm scaling and smoothing (tune s values here before committing to robot)
python -m ship_emulation.analyze_data \
    --linear-scale 1000 \
    --smooth-linear 1e9 \
    --smooth-angular 1e2 \
    --save preview.png

# Dry run (validate full pipeline, no robot)
python -m ship_emulation.run \
    --csv-file ship_emulation/1_vessel_motion_clean.csv \
    --dry-run

# Live run with smoothing
python -m ship_emulation.run \
    --csv-file ship_emulation/1_vessel_motion_clean.csv \
    --robot-ip 172.17.0.2 \
    --smooth-linear 1e9

# Or just edit the __main__ block in run.py and run:
python -m ship_emulation.run
```

---

## Notes

- **`work_frame`** in `RobotConfig` must match your physical setup — it defines the Cartesian origin for CSV pose offsets. CSV data contains relative displacements, so if `work_frame=(0,0,0,0,0,0)` the robot will try to reach absolute coordinates from its base frame, which are unreachable.
- **`home_pose`** in `RobotConfig` is a Cartesian pose in the work frame (default `(0,0,0,0,0,0)`). All motion sequences in `phase1.py` start from zero, so setting `home_pose = (0,0,0,0,0,0)` ensures the emulator's automatic "move to first pose" step is always a no-op. If you need a raised safe position between axes, set `home_pose` accordingly and adjust `SWEEP_START` / `SWEEP_END` to use the same reference frame.
- If the robot cannot keep up with the simulation rate (logged as "late" moves), reduce `SafetyConfig.max_linear_rate` / `max_angular_rate`, or increase `RobotConfig.linear_speed` / `angular_speed` — within safe limits. Alternatively, increase the smoothing factor to reduce peak velocity in the data.
- **Physical E-stop must always be within operator reach when the robot is powered.**
- **CRI/RTDE throughput limit:** `moveL` commands via CRI have ~80–150 ms round-trip overhead per command (send → robot executes → done signal). Effective maximum command rate is ~10–13 Hz. Running `SAMPLE_RATE` above this causes unbounded lateness accumulation. Keep `SAMPLE_RATE = 10.0` for both phase scripts.
- **`blend_radius`:** setting a non-zero blend radius (default 3 mm in `phase1.py`) allows the UR controller to pipeline consecutive `moveL` commands without a full stop at each waypoint. For smooth trajectories (sine, slow ramps) the path deviation is negligible except at amplitude peaks where the robot turns around ~`blend_radius` early. For stop-and-go characterisation set `blend_radius = 0`.
- **Running the scripts:** the venv at `venv-tactip/` must be active. Scripts include a `sys.path` insert so they can be run directly or as modules from the repo root.

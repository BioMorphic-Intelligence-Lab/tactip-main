# Aerial Tactile Servoing — Meta Repository

This repository is the central place for the **Aerial Tactile Servoing (ATS)** project. It aggregates submodules from [DexterousRobot](https://github.com/dexterousrobot) (forked and extended) and contains all code needed to go from raw data collection to a deployed tactile servo controller on a real or simulated robot.

The project demonstrates tactile servoing on an aerial manipulator using the [TacTip](https://softroboticstoolkit.com/book/tactip) optical tactile sensor. A new TacTip embodiment was designed for aerial use, requiring new training data, new models, and a full end-to-end workflow.

---

## Repository structure

| Submodule / folder | Purpose |
|---|---|
| `tactile_servo_control/` | End-to-end pipeline: data collection, model training, evaluation, and servo control |
| `tactile_sim/` | PyBullet physics simulator for robot arms and tactile sensors |
| `tactile_learning/` | CNN and ViT model definitions and training utilities |
| `tactile_image_processing/` | Image pre-processing, augmentation, marker extraction, and sensor interface |
| `common_robot_interface/` | Unified control API for UR, Franka, ABB, and Dobot robots |
| `tactile_data/` | Collected training/validation data and trained model checkpoints |
| `tactile_data_shear/` | Shear-force data and analysis scripts |
| `programs/` | UR robot programs (symlink) |
| `urcaps/` | UR CAPs plugins |
| `docker/` | Docker environment files |

---

## Workflow

The full pipeline runs in the following order:

1. **Simulate & validate** — Use `tactile_sim` to rehearse data collection and servo control in PyBullet before touching real hardware.
2. **Collect data** — Drive the robot over a surface/edge with `launch_collect_data.py` to gather labelled tactile images.
3. **Pre-process images** — Tune crop, normalisation, and augmentation parameters in `tactile_image_processing`.
4. **Correct label shift** — If the sensor position shifted between sessions, apply the label-shift correction utilities.
5. **Train a model** — Run `launch_training.py` (or `launch_hyper_training.py` for hyperparameter search) to fit a CNN to the collected data.
6. **Evaluate** — Use `evaluate_model.py` to check prediction accuracy on the validation set.
7. **Demo the sensor** — Run `sensor_demo.py` for a live visualisation of the model's pose/shear predictions.
8. **Servo control** — Run `launch_servo_control.py` to close the loop and perform surface following, edge following, or other tasks.

---

## Key components

### `tactile_servo_control`
The main application package. Entry points:

| Script | What it does |
|---|---|
| `collect_data/launch_collect_data.py` | Collect labelled images from robot + sensor |
| `learning/launch_training.py` | Train a pose/shear prediction model |
| `learning/launch_hyper_training.py` | Hyperparameter optimisation with Hyperopt |
| `prediction/evaluate_model.py` | Evaluate a trained model on validation data |
| `servo_control/launch_servo_control.py` | Run closed-loop tactile servo control |
| `utils/sensor_demo.py` | Live sensor demo with real-time predictions |

Supported tasks (via `-t`): `surface_3d`, `edge_2d`, `edge_3d`, `edge_5d`, `surface_5d`, `surface_6d`  
Supported robots (via `-r`): `sim`, `ur`, `mg400`, `cr`, `sim_cr`  
Supported sensors (via `-s`): `tactip`, `tactip_127`, `tactip_331`

### `tactile_sim`
PyBullet simulator supporting:
- **Robot arms:** UR5, Franka Panda, Kuka IIWA, Dobot CR3, Dobot MG400
- **Tactile sensors:** TacTip (optical), DIGIT (gel-based), DigiTac (hybrid)
- **Control modes:** TCP position/velocity, joint position/velocity
- **Embodiments:** arm-only, arm + tactile sensor, arm + vision sensor

### `tactile_learning`
Supervised learning models:
- Simple CNN for pose/shear regression
- Pix2Pix for image-to-image translation
- Vision Transformer (ViT) integration

### `tactile_image_processing`
- Image normalisation, cropping, and augmentation
- Marker extraction via blob detection
- Kernel density estimation and Voronoi tessellation
- Real-sensor interface (`simple_sensors.py`)

### `common_robot_interface`
Unified Python API for:
- **UR3/5/10** via RTDE
- **Franka Panda** via libfranka
- **ABB IRB 120** via TCP/IP
- **Dobot Magician / MG400** via TCP/IP
- **Simulated arms** via `tactile_sim`

Also includes a GUI robot jogger (`tools/robot_jogger/`) for manual positioning.

---

## Quick start

```bash
# 1. Clone with all submodules
git clone --recurse-submodules https://github.com/mbrummelhuis/ats-meta.git
cd ats-meta

# 2. Install dependencies
pip install -r requirements.txt

# 3. Live sensor demo (configure paths inside the script for your setup)
python3 tactile_servo_control/tactile_servo_control/utils/sensor_demo.py

# 4. Simulated data collection
python3 tactile_servo_control/tactile_servo_control/collect_data/launch_collect_data.py -r sim -s tactip -t surface_3d

# 5. Train a model
python3 tactile_servo_control/tactile_servo_control/learning/launch_training.py -r sim -s tactip -t surface_3d -d cpu

# 6. Run servo control in simulation
python3 tactile_servo_control/tactile_servo_control/servo_control/launch_servo_control.py -r sim -s tactip -t surface_3d
```

For full installation and usage instructions see the [tactile_servo_control README](https://github.com/mbrummelhuis/tactile_servo_control/tree/readme-maintenance).

---

## Tech stack

- **Deep learning:** PyTorch, ViT-PyTorch, TensorBoard, Hyperopt
- **Computer vision:** OpenCV, scikit-image
- **Robotics / simulation:** PyBullet, ROS 2 (Humble), UR RTDE
- **Numerics:** NumPy, SciPy, Pandas, Einops
- **License:** GPL v3

---

## References

- [Pose-Based Tactile Servoing](https://ieeexplore.ieee.org/document/9502718)
- [Pose- and Shear-based Tactile Servoing](https://arxiv.org/abs/2312.08411)

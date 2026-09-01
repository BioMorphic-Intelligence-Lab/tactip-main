# Aerial Tactile Servoing — Tactip Main Repository

This repository is the central place for the **Aerial Tactile Servoing (ATS)** project. 

The project demonstrates tactile servoing on an aerial manipulator using the [TacTip](https://softroboticstoolkit.com/book/tactip) optical tactile sensor. A new TacTip embodiment was designed for aerial use, requiring new training data, new models, and a full end-to-end workflow.
---

## History

This repository was pieced together from the various codebases previously published under [DexterousRobot](https://github.com/dexterousrobot), and extended to include the `ur_simulation_suite-tactip` and many changes to facilitate inclusion of force in collection and prediction.

The old repositories at [DexterousRobot](https://github.com/dexterousrobot) are no longer public.

---

## Repository structure

| Submodule / folder | Purpose |
|---|---|
| `tactile_servo_control/` | End-to-end pipeline: data collection, model training, evaluation, and servo control |
| `tactile_learning/` | CNN and ViT model definitions and training utilities |
| `tactile_image_processing/` | Image pre-processing, augmentation, marker extraction, and sensor interface |
| `common_robot_interface/` | Unified control API for UR, Franka, ABB, and Dobot robots |
| `tactile_data_shear/` | Shear-force data and analysis scripts |
| `programs/` | UR robot programs (symlink) |
| `urcaps/` | UR CAPs plugins |
| `docker/` | Docker environment files |

---

## Workflow

The full pipeline runs in the following order:

1. **Simulate & validate** — Use `ur_simulation_suite-tactip` to rehearse data collection on a simulated UR16e robot, to validate hardware control.
2. **Collect data** — Drive the robot over a surface/edge with `launch_collect_data.py` to gather labelled tactile images (and, if needed, force samples).
3. **Pre-process images** — Tune crop, normalisation, and augmentation parameters in `tactile_image_processing`.
4. **Correct label shift** — If the sensor position shifted between sessions, apply the label-shift correction utilities.
5. **Train a model** — Run `launch_training.py` to fit a CNN to the collected data.
6. **Evaluate** — Use `evaluate_model.py` to check prediction accuracy on the validation set.
7. **Hyperparameter optimization** — Configure and run `launch_hyper_training.py` to search the defined space for the optimal hyperparameters.
7. **Demo the sensor** — Run `sensor_demo.py` for a live visualisation of the model's pose/shear predictions.
8. **Deploy** — Copy over models for use in the ROS2 based Tactip driver node from the [Aerial Tactile Servoing](https://github.com/BioMorphic-Intelligence-Lab/aerial_tactile_servoing) project.

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
| `utils/sensor_demo.py` | Live sensor demo with real-time predictions |

Supported tasks (via `-t`): `surface_3d`, `edge_2d`, `edge_3d`, `edge_5d`, `surface_5d`, `surface_6d`  
Supported robots (via `-r`): `sim`, `ur`, `mg400`, `cr`, `sim_cr`  
Supported sensors (via `-s`): `tactip`, `tactip_127`, `tactip_331`

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
# 1. Clone this repository
git clone https://github.com/BioMorphic-Intelligence-Lab/tactip-main.git
cd tactip-main

# 2. Install dependencies
pip install -r requirements.txt

# 3. Install internal dependencies
cd tactile_data_shear && pip install -e . && cd ..
cd tactile_image_processing && pip install -e . && cd ..
cd tactile_learning && pip install -e . && cd ..
cd tactile_servo control && pip install -e . && cd ..
cd common_robot_interface && pip install -e . && cd ..

# 3. Live sensor demo (configure paths inside the script for your setup)
python3 tactile_servo_control/tactile_servo_control/utils/sensor_demo.py

# 5. Train a model
python3 tactile_servo_control/tactile_servo_control/learning/launch_training.py -r sim -s tactip -t surface_3d -d cpu
```

## References

- [Pose-Based Tactile Servoing](https://ieeexplore.ieee.org/document/9502718)
- [Pose- and Shear-based Tactile Servoing](https://arxiv.org/abs/2312.08411)


# 9D Force & Pose Estimation Workflow

On this branch, the pipeline is extended to train a **9D tactile state estimation model** predicting contact pose, shear, and 3D contact forces simultaneously:

$$
\mathbf{y} = [z, R_x, R_y, \text{shear}_x, \text{shear}_y, \text{shear}_{Rz}, F_x, F_y, F_z]
$$

### 1. Hardware Check
Test the PhidgetBridge 3-axis load cell connection and calibration:
```bash
python tactile_image_processing/tactile_image_processing/collect_data/test_sensor.py
```

### 2. Data Collection
Collect synchronized tactile images, robot poses, and force readings:
```bash
python tactile_servo_control/tactile_servo_control/collect_data/launch_collect_data.py -r ur -s tactip -t surface_9d -n 3000
```
> **Note**: Adjust camera `source` index, bounding box (`BBOX`), and thresholding in `setup_collect_data.py`.

### 3. Force Coordinate Transformation
Because the table-mounted load cell measures forces in the **World Frame**, convert all force labels ($F_x, F_y, F_z$) into the **local TacTip camera frame** ($F_{\text{tactip}} = R^{-1} F_{\text{world}}$) before training:
```bash
python tactile_servo_control/tactile_servo_control/learning/rotation_matrix.py
```

### 4. Model Training
Train the 9D regression CNN on GPU:
```bash
python tactile_servo_control/tactile_servo_control/learning/launch_training.py -r ur -s tactip -t surface_9d -m simple_cnn -mv B1 -d cuda
```

### 5. Live Model Verification
Run the real-time inference script to test the model with a live OpenCV HUD and compare predictions against the ground-truth Phidget force sensor:
```bash
python tactile_image_processing/tactile_image_processing/collect_data/verification_model.py
```

> **Calibrating Sensor Alignment (`TACTIP_ANGLE_DEG`)**:  
> In `verification_model.py`, `TACTIP_ANGLE_DEG` applies a 2D planar rotation to align the TacTip internal camera axes with the physical reference mark (e.g., tape mark) and the load cell:
> 1. Set `TACTIP_ANGLE_DEG = 0.0` in `verification_model.py` to observe raw unrotated model predictions ($F_{x, \text{cam}}, F_{y, \text{cam}}$).
> 2. **Align Tape Mark**: Place the TacTip on the test plate so your physical Tape Mark points directly along $+Y_{\text{sensor}}$ (Forward).
> 3. **Push along $+Y_{\text{sensor}}$**: Press down and slide the TacTip purely in the $+Y_{\text{sensor}}$ direction.
> 4. **Calculate Rotation Angle**: Read the observed $F_{x, \text{cam}}$ and $F_{y, \text{cam}}$ outputs and compute the angle:  
>    $\theta = \text{arctan2}(F_{x, \text{cam}}, F_{y, \text{cam}})$  
>    The difference between the camera angle and reference load cell angle gives the required rotation offset.
> 5. Update `TACTIP_ANGLE_DEG = <calculated_angle_in_degrees>` in `verification_model.py` so that reported $F_x, F_y$ align with the physical sensor frame.


# References
[Pose-Based Tactile Servoing](https://ieeexplore.ieee.org/document/9502718)

[Pose- and Shear-based Tactile Servoing](https://arxiv.org/abs/2312.08411)

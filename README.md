# Tactip meta repository
This repository was created as a central place for projects involving the TacTip optical tactile sensor. It contains as submodules the relevant repositories from [DexterousRobot](https://github.com/dexterousrobot), which accompany a number of [publications](#references) related to the TacTip. Furthermore, it contains a simulator for the Universal Robots UR5, which can be used to simulate the robot arm interfacing with the rest of the code through Common Robot Interface's RTDE controller. This was used to prepare the data collection for the TacTip, and familiarise and validate the code before letting it control a real life robot.

Note that the submodules of this repository contain forks of the original repositories from [DexterousRobot](https://github.com/dexterousrobot). 

# Project context
While the TacTip as an optical tactile sensor has been used in numerous works, the context of this meta repository is the Aerial Tactile Servoing project. For this, a new embodiment of the TacTip was designed and thus new training data had to be gathered and new models had to be trained. The goal of the project is to demonstrate tactile servoing on an aerial manipulator. The meta repository contains all the work related to this full workflow:
- Preparing robot control for data gathering
- Tuning image (pre-)processing parameters
- Shifting data in case of label shift
- Training pose- and shear prediction models on the training data
- Demoing the sensor with a demo tool



# How to use
If you came to this repository because you want to get started with the TacTip, you can proceed in the following way:
1. Clone this repository recursively 
    ```
    git clone --recurse-submodules https://github.com/mbrummelhuis/ats-meta.git

    ```
2. The demo can be started with
    ```
    cd /root/of/ats-meta
    python3 tactile_servo_control/tactile_servo_control/utils/sensor_demo.py
    ```
    You may need to go into the Python file to configure things correctly for your setup.
3. The README of [tactile_servo_control](https://github.com/mbrummelhuis/tactile_servo_control/tree/readme-maintenance) (readme-maintenance branch) contains instructions on installation of submodules, data collection, learning/training, and using the models. 



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
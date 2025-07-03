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


# References
[Pose-Based Tactile Servoing](https://ieeexplore.ieee.org/document/9502718)

[Pose- and Shear-based Tactile Servoing](https://arxiv.org/abs/2312.08411)
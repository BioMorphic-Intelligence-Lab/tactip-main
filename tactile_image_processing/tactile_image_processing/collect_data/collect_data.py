import os
import numpy as np
import time

from tactile_image_processing.collect_data.setup_embodiment import setup_embodiment
from tactile_image_processing.collect_data.setup_targets import setup_targets
from tactile_image_processing.collect_data.setup_targets import POSE_LABEL_NAMES, SHEAR_LABEL_NAMES, OBJECT_POSE_LABEL_NAMES, FT_LABEL_NAMES
from tactile_image_processing.utils import make_dir, save_json_obj
from tactile_image_processing.collect_data.test_sensor import ThreeAxisForceSensor

BASE_DATA_PATH = 'temp'

def collect_data(
    robot,
    sensor,
    targets_df,
    image_dir,
    collect_params,
):
    pose_label_names = collect_params.get('pose_label_names', POSE_LABEL_NAMES)
    shear_label_names = collect_params.get('shear_label_names', SHEAR_LABEL_NAMES)
    object_pose_label_names = collect_params.get('object_pose_label_names', OBJECT_POSE_LABEL_NAMES)

    # Initialize and Start Phidget Force Sensor
    phidget_sensor = ThreeAxisForceSensor()
    phidget_sensor.start()

    # start 50mm above workframe origin with zero joint 6
    print("Moving to 50 mm above workframe origin")
    robot.move_linear((0, 0, -50, 0, 0, 0))
    robot.move_joints([*robot.joint_angles[:-1], 0])

    # collect reference image
    print(f"Collecting reference image in {image_dir}/image_0.png")
    image_outfile = os.path.join(image_dir, 'image_0.png')
    sensor.process(image_outfile)
    time.sleep(5)

    # clear object by 10mm
    print("Moving to 10 mm above workframe origin")
    clearance = (0, 0, 10, 0, 0, 0)
    robot.move_linear(np.zeros(6) - clearance)
    joint_angles = robot.joint_angles
    saved_obj_label = ''

    # Global Zero-Tare Routine for Ground Sensor 
    print("\nTaring Phidget force sensor baseline offsets... Do not touch table.")
    time.sleep(1.5)  # Allow any mechanical vibrations to settle
    samples_x, samples_y, samples_z = [], [], []
    for _ in range(20):
        fx, fy, fz = phidget_sensor.get_forces_in_newtons()
        samples_x.append(fx)
        samples_y.append(fy)
        samples_z.append(fz)
        time.sleep(0.02)
    tare = {}
    tare['x'] = np.mean(samples_x)
    tare['y'] = np.mean(samples_y)
    tare['z'] = np.mean(samples_z)

    print("Ground sensor tare calibration complete.\n")

    # Prompt user to physically mark the TacTip's positive X and Y directions
    print("Mark Tactip positive X and Y on the sensor housing. \n")
    input("Press Enter to continue...")

    # ==== data collection loop ====
    print("Starting data collection sequence")
    for i, row in targets_df.iterrows():
        image_name = row.loc["sensor_image"]
        obj_label = row.loc["object_label"]
        pose = row.loc[pose_label_names].values.astype(float)
        shear = row.loc[shear_label_names].values.astype(float)
        obj_pose = row.loc[object_pose_label_names].values.astype(float)
        print("Object pose: ", obj_pose)

        # report
        with np.printoptions(precision=1, suppress=True):
            print(f"{i+1}/{len(targets_df.index)}: [{obj_label}] pose{pose}, shear{shear}")

        # new object set reset
        if obj_label != saved_obj_label:
            saved_obj_label = obj_label
            robot.move_joints(joint_angles)
            robot.move_linear(obj_pose - clearance)
            joint_angles = robot.joint_angles

        # pose is relative to object pose
        pose += obj_pose

        # move to above new pose (avoid changing pose in contact with object)
        robot.move_linear(pose + shear - clearance)

        # this is for UR sensor!
        #print("Zeroing FT sensor in mid-air...")
        #time.sleep(1.0)         # Wait for vibrations to settle
        #robot.zero_ft_sensor()  # Send command 14
        #time.sleep(0.2)         # Short pause for the zeroing to register
 
        print("Approaching target pose...")
        time.sleep(0.2)

        # move down to offset pose
        robot.move_linear(pose + shear)

        # move to target pose inducing shear
        robot.move_linear(pose)

        time.sleep(0.3)  # 300ms settling window ensures static equilibrium

        # collect and process tactile image
        image_outfile = os.path.join(image_dir, image_name)
        sensor.process(image_outfile)

        # force/torque reading - average multiple samples to reduce noise
        num_samples = 30
        samples = []
        
        for _ in range(num_samples):
            # this is for UR sensor!
            #samples.append(robot.get_tcp_force)

            # Read from Phidget sensor, apply zero-tare offset, and shape array
            raw_fx, raw_fy, raw_fz = phidget_sensor.get_forces_in_newtons()
            net_fx = raw_fx - tare['x']
            net_fy = raw_fy - tare['y']
            net_fz = raw_fz - tare["z"]
            
            # Map values to match the [Fx, Fy, Fz, Tx, Ty, Tz] shape expected by targets_df
            samples.append([net_fx, net_fy, net_fz, 0.0, 0.0, 0.0])
            time.sleep(0.005) # Faster polling rate for structural cohesion

            
        force_torque = np.mean(np.array(samples), axis=0)
        
        print(f" Averaged Force/Torque ({num_samples} samples): ", force_torque)

        for j, col in enumerate(FT_LABEL_NAMES):
            targets_df.at[i, col] = force_torque[j]

        # move above the target pose
        robot.move_linear(pose - clearance)

        # if sorted, don't move to reset position
        if not collect_params.get('sort', False):
            robot.move_joints(joint_angles)

    # finish 100mm above workframe origin then zero last joint
    robot.move_linear((0, 0, -100, 0, 0, 0))
    robot.move_joints((*robot.joint_angles[:-1], 0))
    
    # Safely close both handlers
    phidget_sensor.close()
    robot.close()

    # overwrite targets.csv with updated force/torque values
    save_dir = os.path.dirname(image_dir)
    target_file = os.path.join(save_dir, "targets.csv")
    targets_df.to_csv(target_file, index=False)
    print(f"Updated targets saved to {target_file}")


if __name__ == "__main__":

    data_params = {
        'data_1': 50,
        'data_2': 50,
    }

    collect_params = {
        "pose_llims": (-5, 0, 3, 0, 0, -180),
        "pose_ulims": (5, 0, 4, 0, 0,  180),
        "sort": True,
        "object_poses": {
            "edge":    (0, 0, 0, 0, 0, 0),
            "surface": (-50, 0, 0, 0, 0, 0)
        }
    }

    env_params = {
        "robot": "sim",
        "stim_name": "square",
        "work_frame": (650, 0, 50, -180, 0, 0),
        "tcp_pose":   (0, 0, -85, 0, 0, 0),
        "stim_pose":  (600, 0, 12.5, 0, 0, 0),
        'show_tactile': True
    }

    sensor_params = {
        "type": "standard_tactip",
        "image_size": (256, 256)
    }

    for data_dir_name, num_poses in data_params.items():

        # setup save dir
        save_dir = os.path.join(BASE_DATA_PATH, data_dir_name)
        image_dir = os.path.join(save_dir, "sensor_images")
        make_dir(save_dir)
        make_dir(image_dir)
        save_json_obj(sensor_params, os.path.join(save_dir, 'sensor_image_params'))

        # setup embodiment
        robot, sensor = setup_embodiment(
            env_params,
            sensor_params
        )

        # setup targets to collect
        target_df = setup_targets(
            collect_params,
            num_poses,
            save_dir
        )

        # collect
        collect_data(
            robot,
            sensor,
            target_df,
            image_dir,
            collect_params
        )

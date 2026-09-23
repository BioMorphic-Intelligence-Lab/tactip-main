import os
import sys
import numpy as np
import time

from tactile_image_processing.collect_data.setup_embodiment import setup_embodiment
from tactile_image_processing.collect_data.setup_targets import setup_targets
from tactile_image_processing.collect_data.setup_targets import POSE_LABEL_NAMES, SHEAR_LABEL_NAMES, OBJECT_POSE_LABEL_NAMES, FT_LABEL_NAMES
from tactile_image_processing.utils import make_dir, save_json_obj

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
    collect_force = collect_params.get('collect_force', False)

    phidget_sensor = None
    tare_x, tare_y, tare_z = 0.0, 0.0, 0.0

    # 1. Initialize Phidget sensor if force collection is requested
    if collect_force:
        try:
            from tactile_image_processing.collect_data.test_sensor import ThreeAxisForceSensor
            phidget_sensor = ThreeAxisForceSensor()
            phidget_sensor.start()
            print("[INFO] Successfully connected to Phidget 3-Axis Force Sensor.")
        except Exception as e:
            print(f"\n[ERROR] Could not connect to Phidget Force Sensor: {e}")
            print("[ERROR] Please verify the USB connection or set collect_force=False.\n")
            sys.exit(1)

    # 2. Camera verification check
    test_img = os.path.join(image_dir, 'connection_test.png')
    try:
        sensor.process(test_img)
        if os.path.exists(test_img):
            os.remove(test_img)
    except Exception as e:
        print(f"\n[ERROR] Tactile camera connection error: {e}")
        print("[ERROR] Please check USB camera port/index in sensor_image_params.\n")
        if phidget_sensor:
            phidget_sensor.close()
        sys.exit(1)

    # 3. Position above workframe
    print("Moving to 50 mm above workframe origin")
    robot.move_linear((0, 0, -50, 0, 0, 0))
    robot.move_joints([*robot.joint_angles[:-1], 0])

    # 4. Reference image collection
    print(f"Collecting reference image in {image_dir}/image_0.png")
    image_outfile = os.path.join(image_dir, 'image_0.png')
    sensor.process(image_outfile)
    time.sleep(5)

    # 5. Clearance offset
    print("Moving to 10 mm above workframe origin")
    clearance = (0, 0, 10, 0, 0, 0)
    robot.move_linear(np.zeros(6) - clearance)
    joint_angles = robot.joint_angles
    saved_obj_label = ''

    # 6. Tare Phidget baseline offsets
    if collect_force and phidget_sensor:
        print("\nTaring Phidget force sensor baseline offsets... Do not touch table.")
        time.sleep(120)
        samples_x, samples_y, samples_z = [], [], []
        for _ in range(20):
            fx, fy, fz = phidget_sensor.get_forces_in_newtons()
            samples_x.append(fx)
            samples_y.append(fy)
            samples_z.append(fz)
            time.sleep(0.02)
        tare_x = np.mean(samples_x)
        tare_y = np.mean(samples_y)
        tare_z = np.mean(samples_z)
        print("Ground sensor tare calibration complete.\n")

    # 7. Data collection loop
    print("Starting data collection sequence")
    try:
        for i, row in targets_df.iterrows():
            image_name = row.loc["sensor_image"]
            obj_label = row.loc["object_label"]
            pose = row.loc[pose_label_names].values.astype(float)
            shear = row.loc[shear_label_names].values.astype(float)
            obj_pose = row.loc[object_pose_label_names].values.astype(float)

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

            print("Approaching target pose...")
            time.sleep(0.2)

            # move down to offset pose
            robot.move_linear(pose + shear)

            # move to target pose inducing shear
            robot.move_linear(pose)

            time.sleep(0.3)  # 300ms settling window ensures static equilibrium

            # Capture image
            image_outfile = os.path.join(image_dir, image_name)
            sensor.process(image_outfile)

            # Sample external Phidget force readings
            if collect_force:
                num_samples = 30
                samples = []
                for _ in range(num_samples):
                    raw_fx, raw_fy, raw_fz = phidget_sensor.get_forces_in_newtons()
                    net_fx = raw_fx - tare_x
                    net_fy = raw_fy - tare_y
                    net_fz = raw_fz - tare_z
                    samples.append([net_fx, net_fy, net_fz, 0.0, 0.0, 0.0])
                    time.sleep(0.005)

                force_torque = np.mean(np.array(samples), axis=0)
                print(f" Averaged Force/Torque ({num_samples} samples): ", force_torque)
                for j, col in enumerate(FT_LABEL_NAMES):
                    targets_df.at[i, col] = force_torque[j]

            robot.move_linear(pose - clearance)

            # if sorted, don't move to reset position
            if not collect_params.get('sort', False):
                robot.move_joints(joint_angles)

    finally:
        # Safe robot & sensor teardown
        robot.move_linear((0, 0, -100, 0, 0, 0))
        robot.move_joints((*robot.joint_angles[:-1], 0))
        if phidget_sensor:
            phidget_sensor.close()
        robot.close()

    # Save target CSV
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

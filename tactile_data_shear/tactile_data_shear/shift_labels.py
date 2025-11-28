import pandas as pd
import os

from tactile_data_shear.tactile_servo_control import BASE_DATA_PATH
from tactile_image_processing.process_data.process_image_data import process_image_data, partition_data

BBOX = { # (x0, y0, x1, y1)
    "abb_tactip":   (25, 25, 305, 305),
    "cr_tactip":    (5, 10, 425, 430),
    "mg400_tactip": (10, 10, 310, 310),
    "ur_tactip":    (60, 65, 410, 415), # We just use the full resolution and set it in bbox_dict below
    "sim_tactip":   (12, 12, 240, 240)
}
CIRCLE_MASK_RADIUS = {
    "abb_tactip":   140,
    "cr_tactip":    210,
    "mg400_tactip": None,
    "ur_tactip":    170,
    "sim_tactip":   240
}
THRESH = {
    "abb_tactip":   [61, 5],
    "cr_tactip":    [61, 5],
    "mg400_tactip": [61, 5],
    "ur_tactip":    [61, -8.5], # Determined using tune_images.py in tactile_image_processing
    "sim_tactip":   None
}

import argparse


def parse_args(
        robot='sim',
        sensor='tactip',
        tasks=['edge_2d'],
        data_dirs=['train', 'val'],
        sample_nums=[400, 100],
        train_dirs=['train'],
        val_dirs=['val'],
        models=['simple_cnn'],
        model_version=[],
        objects=['circle'],
        run_version=[],
        device='cuda'
):
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '-r', '--robot',
        type=str,
        help="Choose robot from ['sim', 'mg400', 'cr']",
        default=robot
    )
    parser.add_argument(
        '-s', '--sensor',
        type=str,
        help="Choose sensor from ['tactip', 'tactip_127']",
        default=sensor
    )
    parser.add_argument(
        '-t', '--tasks',
        nargs='+',
        help="Choose tasks from ['surface_3d', 'edge_2d', 'edge_3d', 'edge_5d']",
        default=tasks
    )
    parser.add_argument(
        '-dd', '--data_dirs',
        nargs='+',
        help="Specify data directories (default ['train', 'val']).",
        default=data_dirs
    )
    parser.add_argument(
        '-n', '--sample_nums',
        type=int,
        help="Choose numbers of samples (default [400, 100]).",
        default=sample_nums
    )
    parser.add_argument(
        '-dt', '--train_dirs',
        nargs='+',
        help="Specify train data directories (default ['train').",
        default=train_dirs
    )
    parser.add_argument(
        '-dv', '--val_dirs',
        nargs='+',
        help="Specify validation data directories (default ['val']).",
        default=val_dirs
    )
    parser.add_argument(
        '-m', '--models',
        nargs='+',
        help="Choose models from ['simple_cnn', 'posenet_cnn', 'nature_cnn', 'resnet', 'vit']",
        default=models
    )
    parser.add_argument(
        '-mv', '--model_version',
        type=str,
        help="Choose version.",
        default=model_version
    )
    parser.add_argument(
        '-o', '--objects',
        nargs='+',
        help="Choose objects from ['circle', 'square', 'clover', 'foil', 'saddle', 'bowl']",
        default=objects
    )
    parser.add_argument(
        '-rv', '--run_version',
        type=str,
        help="Choose version.",
        default=run_version
    )
    parser.add_argument(
        '-d', '--device',
        type=str,
        help="Choose device from ['cpu', 'cuda']",
        default=device
    )

    return parser.parse_args()


def reprocess_data(args, image_params, split=0.8):
    output_dir = '_'.join([args.robot, args.sensor])

    for args.task in args.tasks:
        path = os.path.join(BASE_DATA_PATH, output_dir, args.task)
        data_dirs = partition_data(path, args.data_dirs, split)
        process_image_data(path, data_dirs, image_params)

def main(args, shift=1):
    """Simple script to shift the labels in the dataset if during collection unintentionally the labels were shifted.
    It simply emulates the data processing that happens at the end of the data collection script.

    Args:
        args (arg): arguments of the data collection (replication of the data collection script)
    """
    # set the paths (change here for your data)
    full_data_path = os.path.join(BASE_DATA_PATH,args.robot+'_'+args.sensor, args.tasks[0], 'data')
    full_targets_path = os.path.join(full_data_path, 'targets.csv')

    target_path = os.path.join(BASE_DATA_PATH, 'ur_tactip', 'surface_3d', 'data', 'targets.csv')

    print(f"WARNING: This operation will modify data stored at {full_data_path}")
    print(f"Labels will be shifted by {shift} and the data will be reprocessed. Backing up original data is advised.")
    print("Press Enter to continue or Ctrl+C to cancel")
    input()

    # read original labels and shift them
    df = pd.read_csv(full_targets_path)
    df['sensor_image'] = df['sensor_image'].shift(shift)
    
    # drop the rows with NaN values and save
    df = df.dropna()
    df.to_csv(target_path, index=False)
    
    # for checking manually if the result is as intended
    print(df.head(5))
    print(df.tail(5))

    # reprocess the data - copy of data collection script
    embodiment = '_'.join([args.robot, args.sensor])
    image_params = {
        "bbox": BBOX[embodiment],
        "circle_mask_radius": CIRCLE_MASK_RADIUS[embodiment],
        "thresh": THRESH[embodiment],
    }
    reprocess_data(args, image_params, split=0.8)

if __name__ == "__main__":
    args = parse_args(
            robot='ur',
            sensor='tactip',
            tasks=['surface_9d'],
            data_dirs=['data'],
            sample_nums=[4000]
        )
    main(args, shift = 0)
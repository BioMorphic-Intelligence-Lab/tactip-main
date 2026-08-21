import os
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

BASE_PATH = "tactile_data_shear/data/ur_tactip/surface_9d"
SUBFOLDERS = ["data", "train_data", "val_data"]
TARGET_FILENAMES = ["targets.csv", "targets_images.csv"]

CREATE_BACKUP = False 

def transform_forces_to_tactip_frame(file_path):
    if not os.path.exists(file_path):
        return 
    
    df = pd.read_csv(file_path)
    
    required_cols = ['pose_Rx', 'pose_Ry', 'pose_Rz', 'Fx', 'Fy', 'Fz']
    if not all(col in df.columns for col in required_cols):
        print(f"Skipping {file_path}: Missing required columns.")
        return

    if CREATE_BACKUP:
        backup_path = file_path.replace(".csv", "_raw_backup.csv")
        if not os.path.exists(backup_path):
            df.to_csv(backup_path, index=False)
            print(f"  └─ Created backup: {backup_path}")

    eulers = df[['pose_Rx', 'pose_Ry', 'pose_Rz']].to_numpy()
    rotations = R.from_euler('xyz', eulers, degrees=True)
    
    forces_world = df[['Fx', 'Fy', 'Fz']].to_numpy()
    
    # Vectorized Inverse Rotation: F_tactip = R^-1 * F_world
    forces_tactip = rotations.inv().apply(forces_world)
    
    df['Fx'] = forces_tactip[:, 0]
    df['Fy'] = forces_tactip[:, 1]
    df['Fz'] = forces_tactip[:, 2]
    
    df.to_csv(file_path, index=False)
    print(f"Successfully updated: {file_path}")

def main():
    print("Starting coordinate transformation (Table Frame -> TacTip Frame)...\n")
    processed_count = 0
    
    for folder in SUBFOLDERS:
        folder_path = os.path.join(BASE_PATH, folder)
        for filename in TARGET_FILENAMES:
            full_path = os.path.join(folder_path, filename)
            if os.path.exists(full_path):
                transform_forces_to_tactip_frame(full_path)
                processed_count += 1

    print(f"\nFinished processing {processed_count} files.")
    print("All Fx, Fy, Fz values are now expressed in the local TacTip camera frame.")

if __name__ == "__main__":
    main()
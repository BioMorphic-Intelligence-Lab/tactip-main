import pandas as pd
import numpy as np
import os
from scipy.spatial.transform import Rotation as R

base_path = "tactile_data_shear/data/ur_tactip/surface_9d"
subfolders = ["data", "train_data", "val_data"]
target_filenames = ["targets.csv", "targets_images.csv"]

def rotate_inplace(file_path):
    if not os.path.exists(file_path):
        return # Skip quietly if targets_images doesn't exist
    
    df = pd.read_csv(file_path)
    
    required_cols = ['pose_Rx', 'pose_Ry', 'pose_Rz', 'Fx', 'Fy', 'Fz']
    if not all(col in df.columns for col in required_cols):
        print(f"Skipping {file_path}: Missing one or more required columns {required_cols}")
        return

    def rotate_row(row):
        # orientation: Euler XYZ in degrees
        euler = [row['pose_Rx'], row['pose_Ry'], row['pose_Rz']]
        rot = R.from_euler('xyz', euler, degrees=True)
        
        f_base = np.array([row['Fx'], row['Fy'], row['Fz']])
        
        # Transform to TCP Frame (Inverse rotation)
        # F_tcp = R^-1 * F_base
        f_tcp = rot.inv().apply(f_base)
        
        row['Fx'], row['Fy'], row['Fz'] = f_tcp[0], f_tcp[1], f_tcp[2]
        return row

    print(f"Processing: {file_path}")
    df = df.apply(rotate_row, axis=1)
    
    # overwrite
    df.to_csv(file_path, index=False)

for folder in subfolders:
    folder_path = os.path.join(base_path, folder)
    for filename in target_filenames:
        full_path = os.path.join(folder_path, filename)
        rotate_inplace(full_path)

print("\nSuccess: All targets.csv and targets_images.csv files updated with TCP forces.")
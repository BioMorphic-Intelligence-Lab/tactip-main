import os
import numpy as np
import pandas as pd


POSE_LABEL_NAMES = ["pose_x", "pose_y", "pose_z", "pose_Rx", "pose_Ry", "pose_Rz"]
SHEAR_LABEL_NAMES = ["shear_x", "shear_y", "shear_z", "shear_Rx", "shear_Ry", "shear_Rz"]
OBJECT_POSE_LABEL_NAMES = ["object_x", "object_y", "object_z", "object_Rx", "object_Ry", "object_Rz"]
FT_LABEL_NAMES = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]


def setup_targets(
   collect_params,
   num_poses=100,
   save_dir=None,
):
   """
   Generates a dataframe with target poses used for data collection.
   """
   pose_label_names = collect_params.get('pose_label_names', POSE_LABEL_NAMES)
   pose_llims = collect_params.get('pose_llims', [0]*6)
   pose_ulims = collect_params.get('pose_ulims', [0]*6)

   shear_label_names = collect_params.get('shear_label_names', SHEAR_LABEL_NAMES)
   shear_llims = collect_params.get('shear_llims', [0]*6)
   shear_ulims = collect_params.get('shear_ulims', [0]*6)

   sample_disk = collect_params.get('sample_disk', False)
   sort = collect_params.get('sort', False)
   collect_force = collect_params.get('collect_force', False)

   object_pose_label_names = collect_params.get('object_pose_label_names', OBJECT_POSE_LABEL_NAMES)
   object_poses_dict = collect_params.get('object_poses', {'1': [0]*6})

   # Base columns; only append force labels if collect_force is True
   columns = [
       "sensor_image",
       "object_label",
       *pose_label_names,
       *shear_label_names,
       *object_pose_label_names,
   ]
   if collect_force:
       columns.extend(FT_LABEL_NAMES)

   target_df = pd.DataFrame(columns=columns)

   np.random.seed(collect_params.get('seed', None))

   ind = 0
   for obj_label, obj_pose in object_poses_dict.items():
       poses = sample_poses(pose_llims, pose_ulims, num_poses, sample_disk)
       shears = sample_poses(shear_llims, shear_ulims, num_poses, sample_disk)

       if sort:
           i_sort = -1 if isinstance(sort, bool) else pose_label_names.index(sort)
           poses = poses[poses[:, i_sort].argsort()]

       for i in range(num_poses):
           image_name = f"image_{ind+1}.png"
           pose = poses[i, :]
           shear = shears[i, :]

           row_data = [image_name, obj_label, *pose, *shear, *obj_pose]
           if collect_force:
               row_data.extend([0.0] * len(FT_LABEL_NAMES))

           target_df.loc[ind] = row_data
           ind += 1

   if save_dir:
       target_file = os.path.join(save_dir, "targets.csv")
       target_df.to_csv(target_file, index=False)

   return target_df


def random_spherical(num_samples, phi_max):
   phi_max = np.radians(phi_max)
   theta = 2 * np.pi * np.random.rand(num_samples)
   kappa = 0.5 * (1 - np.cos(phi_max))
   phi = np.arccos(1 - 2 * kappa * np.random.rand(num_samples))

   x = np.sin(phi) * np.cos(theta)
   y = np.sin(phi) * np.sin(theta)
   z = np.cos(phi)

   Rx = -np.arcsin(y)
   Ry = -np.arctan2(x, z)
   return np.degrees(Rx), np.degrees(Ry)


def random_disk(num_samples, r_max):
   theta = 2 * np.pi * np.random.rand(num_samples)
   r = r_max * np.sqrt(np.random.rand(num_samples))
   x, y = r * (np.cos(theta), np.sin(theta))
   return x, y


def random_linear(num_samples, x_max):
   return -x_max + 2 * x_max * np.random.rand(num_samples)


def sample_poses(llims, ulims, num_samples, sample_disk):
   poses_mid = (np.array(ulims) + llims) / 2
   poses_max = ulims - poses_mid

   samples = [random_linear(num_samples, x_max) for x_max in poses_max]

   if sample_disk:
       inds_pos = [i for i, v in enumerate(poses_max[:2]) if v > 0]
       inds_rot = [3+i for i, v in enumerate(poses_max[3:5]) if v > 0]

       if len(inds_pos) == 2:
           r_max = max(poses_max[inds_pos])
           samples_pos = random_disk(num_samples, r_max)
           scales = poses_max[inds_pos] / r_max
           samples_pos *= scales[np.newaxis, :2].T
           samples[inds_pos[0]], samples[inds_pos[1]] = samples_pos

       if len(inds_rot) == 2:
           phi_max = max(poses_max[inds_rot])
           samples_rot = random_spherical(num_samples, phi_max)
           scales = poses_max[inds_rot[:2]] / phi_max
           samples_rot *= scales[np.newaxis, :].T
           samples[inds_rot[0]], samples[inds_rot[1]] = samples_rot

   poses = np.array(samples).T
   poses += poses_mid

   return poses
import os
import json
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

from tactile_data_shear.tactile_servo_control import BASE_DATA_PATH

"""
Script to analyse the collected training and validation data for the surface_3d task.
"""

def find_most_similar_image(df, target_image, label_columns=None, perturb_columns=None, distance_metric='euclidean'):
    """
    Identifies the most similar image to a target image based on specified columns.
    
    Parameters:
    - df: DataFrame containing image data, with label and perturb columns and an 'image_name' column.
    - target_image: Name of the target image to compare others against.
    - label_columns: List of columns representing labels to compare (optional).
    - perturb_columns: List of columns representing perturbing variables to compare (optional).
    - distance_metric: Metric for distance calculation (default is 'euclidean').
    
    Returns:
    - Name of the most similar image and its distance to the target image.
    """
    # Check that at least one of label_columns or perturb_columns is provided
    if not label_columns and not perturb_columns:
        raise ValueError("At least one of label_columns or perturb_columns must be specified.")
    
    # Combine columns based on specified inputs
    feature_columns = []
    if label_columns:
        feature_columns += label_columns
    if perturb_columns:
        feature_columns += perturb_columns

    # Extract target row and check if it exists in DataFrame
    target_row = df[df['sensor_image'] == target_image][feature_columns]
    if target_row.empty:
        raise ValueError("Target image not found in the dataframe.")
    
    # Compute distances between target and all other rows on specified features
    distances = cdist(target_row, df[feature_columns], metric=distance_metric).flatten()
    
    # Exclude the target image itself by setting its distance to a large value
    target_index = df.index[df['sensor_image'] == target_image][0]
    distances[target_index] = float('inf')
    
    # Find the index of the most similar image (smallest distance)
    most_similar_index = distances.argmin()
    most_similar_image = df.iloc[most_similar_index]['sensor_image']
    most_similar_distance = distances[most_similar_index]
    
    return most_similar_image, most_similar_distance

def main(task='surface_3d'):
    # set paths to the data
    train_data_path = os.path.join(BASE_DATA_PATH, 'ur_tactip', 'surface_3d', 'train_data')
    train_images_path = os.path.join(train_data_path, 'processed_images')
    val_data_path = os.path.join(BASE_DATA_PATH, 'ur_tactip', 'surface_3d', 'val_data')
    val_images_path = os.path.join(val_data_path, 'processed_images')
    
    # get params from json files
    data_collect_params = json.load(open(os.path.join(train_data_path, 'collect_params.json')))

    # load the target csv files
    train_targets = pd.read_csv(os.path.join(train_data_path, 'targets_images.csv'))
    val_targets = pd.read_csv(os.path.join(val_data_path, 'targets_images.csv'))
    
    if task=='surface_3d':
        labels = ['pose_z', 'pose_Rx', 'pose_Ry']
        perturb_vars = ['shear_x', 'shear_y', 'shear_Rz']

    def inspect_label_distribution_training(labels=labels, perturb_vars=perturb_vars):
        """Inspect the distribution of the labels.
        """ 
        fig, axes = plt.subplots(2, 3, figsize=(15, 10), squeeze=False)
        fig.suptitle('Distribution of labels and shear in trainig data')
        
        # Plot the label distributions in the first row
        for i, label in enumerate(labels):
            ax = axes[0, i]  # Select the subplot in the first row
            ax.hist(train_targets[label], bins=30, edgecolor='black')
            ax.set_title(f'Distribution of {label}')
            ax.set_xlabel(label)
            ax.set_ylabel('Count')

        # Plot the perturbing variable distributions in the second row
        for i, perturb in enumerate(perturb_vars):
            ax = axes[1, i]  # Select the subplot in the second row
            ax.hist(train_targets[perturb], bins=30, edgecolor='black')
            ax.set_title(f'Distribution of {perturb}')
            ax.set_xlabel(perturb)
            ax.set_ylabel('Count')
            
        # Adjust layout to prevent overlap
        plt.tight_layout(rect=[0, 0, 1, 0.96])  # Leave space for the main title
        plt.show()

    def inspect_label_distribution_validation(labels=labels, perturb_vars=perturb_vars):
        """Inspect the distribution of the labels.
        """ 
        fig, axes = plt.subplots(2, 3, figsize=(15, 10), squeeze=False)
        fig.suptitle('Distribution of labels and shear in validation data')
        
        # Plot the label distributions in the first row
        for i, label in enumerate(labels):
            ax = axes[0, i]  # Select the subplot in the first row
            ax.hist(val_targets[label], bins=30, edgecolor='black')
            ax.set_title(f'Distribution of {label}')
            ax.set_xlabel(label)
            ax.set_ylabel('Count')

        # Plot the perturbing variable distributions in the second row
        for i, perturb in enumerate(perturb_vars):
            ax = axes[1, i]  # Select the subplot in the second row
            ax.hist(val_targets[perturb], bins=30, edgecolor='black')
            ax.set_title(f'Distribution of {perturb}')
            ax.set_xlabel(perturb)
            ax.set_ylabel('Count')
            
        # Adjust layout to prevent overlap
        plt.tight_layout(rect=[0, 0, 1, 0.96])  # Leave space for the main title
        plt.show()
        
    #inspect_label_distribution_training()
    #inspect_label_distribution_validation()
    
    target_image = 'image_3201.png'
    print(f"Analysing image: {target_image}")
    print(f"Data: {train_targets[train_targets['sensor_image'] == target_image][labels+perturb_vars]}")
    print("")
    # Find the most similar image based only on labels
    similar_image_labels, distance_labels = find_most_similar_image(
        train_targets, target_image=target_image, label_columns=labels
    )
    print(f"Most similar image based on labels: {similar_image_labels}, Distance: {distance_labels}")
    print(f"Data: {train_targets[train_targets['sensor_image'] == similar_image_labels][labels+perturb_vars]}")
    print("")

    # Find the most similar image based only on perturbing variables
    similar_image_perturbs, distance_perturbs = find_most_similar_image(
        train_targets, target_image=target_image, perturb_columns=perturb_vars
    )
    print(f"Most similar image based on perturbing variables: {similar_image_perturbs}, Distance: {distance_perturbs}")
    print(f"Data: {train_targets[train_targets['sensor_image'] == similar_image_perturbs][labels+perturb_vars]}")
    print("")

    # Find the most similar image based on both labels and perturbing variables
    similar_image_both, distance_both = find_most_similar_image(
        train_targets, target_image=target_image, label_columns=labels, 
        perturb_columns=perturb_vars
    )
    print(f"Most similar image based on both labels and perturbing variables: {similar_image_both}, Distance: {distance_both}")
    print(f"Data: {train_targets[train_targets['sensor_image'] == similar_image_both][labels+perturb_vars]}")
    print("")
    
    selected_columns = ['sensor_image', 'pose_x', 'pose_y', 'pose_z', 'pose_Rx', 'pose_Ry', 'pose_Rz', 'shear_x', 'shear_y', 'shear_z', 'shear_Rx', 'shear_Ry', 'shear_Rz']
    topx_data = train_targets.nlargest(5, 'pose_Rx')
    print("Highest values for pose_Rx")
    print(topx_data[selected_columns])
    print("")
        
    topy_data = train_targets.nlargest(5, 'pose_Ry')
    print("Highest values for pose_Ry")
    print(topy_data[selected_columns])
    print("")


if __name__ == '__main__':
    
    main()
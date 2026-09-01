"""
Run best_model.pth on the surface_9d val dataset and export per-sample
predictions vs. targets to val_analytics.csv.

Usage:
    python eval_metrics.py
"""
import os
import numpy as np
import pandas as pd
import torch

from tactile_data_shear.tactile_data_shear.tactile_servo_control_paths import BASE_DATA_PATH, BASE_MODEL_PATH
from tactile_image_processing.utils import load_json_obj
from tactile_learning.supervised.models import create_model
from tactile_learning.supervised.image_generator import ImageDataGenerator
from tactile_servo_control.learning.setup_training import csv_row_to_label
from tactile_servo_control.utils.label_encoder import LabelEncoder

MODEL_DIR = os.path.join(BASE_MODEL_PATH, 'ur_aerial-A1', 'surface_5d', 'A1_2026_hypopt')
VAL_DATA_DIR = os.path.join(BASE_DATA_PATH, 'ur_aerial-A1', 'surface_5d', 'val_data')


def run_inference(device):
    learning_params = load_json_obj(os.path.join(MODEL_DIR, 'learning_params'))
    model_params = load_json_obj(os.path.join(MODEL_DIR, 'model_params'))
    label_params = load_json_obj(os.path.join(MODEL_DIR, 'model_label_params'))
    image_params = load_json_obj(os.path.join(MODEL_DIR, 'model_image_params'))

    label_encoder = LabelEncoder(label_params, device=device)

    generator = ImageDataGenerator(
        [VAL_DATA_DIR],
        csv_row_to_label,
        **image_params['image_processing'],
    )

    loader = torch.utils.data.DataLoader(
        generator,
        batch_size=learning_params['batch_size'],
        shuffle=False,
        num_workers=learning_params['n_cpu'],
    )

    model = create_model(
        in_dim=image_params['image_processing']['dims'],
        in_channels=1,
        out_dim=label_encoder.out_dim,
        model_params=model_params,
        saved_model_dir=MODEL_DIR,
        device=device,
    )
    model.eval()

    target_labels = label_encoder.target_label_names
    pred_rows, targ_rows = [], []

    with torch.no_grad():
        for batch in loader:
            inputs = batch['inputs'].float().to(device)
            outputs = model(inputs)
            pred_dict = label_encoder.decode_label(outputs)
            targ_dict = batch['labels']
            pred_rows.append({k: pred_dict[k].numpy() for k in target_labels})
            targ_rows.append({k: np.asarray(targ_dict[k]) for k in target_labels})

    def stack(rows):
        return pd.DataFrame({k: np.concatenate([r[k] for r in rows]) for k in rows[0]})

    return stack(pred_rows), stack(targ_rows), target_labels


def build_analytics(pred_df, targ_df, target_labels):
    cols = {}
    for label in target_labels:
        cols[f'targ_{label}'] = targ_df[label].values.astype(float)
        cols[f'pred_{label}'] = pred_df[label].values.astype(float)
    return pd.DataFrame(cols)


def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"device: {device}")

    pred_df, targ_df, target_labels = run_inference(device)

    analytics_df = build_analytics(pred_df, targ_df, target_labels)
    analytics_path = os.path.join(MODEL_DIR, 'val_analytics.csv')
    analytics_df.to_csv(analytics_path, index_label='sample_idx')
    print(f"saved: {analytics_path}")


if __name__ == '__main__':
    main()

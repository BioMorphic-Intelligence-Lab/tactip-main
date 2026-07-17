"""
Compute error summary statistics from val_analytics.csv and save to val_summary.csv.

Dependencies: numpy, pandas (no project-specific packages required).

Usage:
    python summarize_metrics.py
"""
import os
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ANALYTICS_PATH = os.path.join(HERE, 'val_analytics.csv')
SUMMARY_PATH = os.path.join(HERE, 'val_summary.csv')


def compute_summary(df):
    labels = [c.removeprefix('targ_') for c in df.columns if c.startswith('targ_')]
    rows = []
    for label in labels:
        targ = df[f'targ_{label}'].values.astype(float)
        pred = df[f'pred_{label}'].values.astype(float)
        err = pred - targ
        abs_err = np.abs(err)
        ss_res = np.sum(err ** 2)
        ss_tot = np.sum((targ - targ.mean()) ** 2)
        rows.append({
            'label': label,
            'mae':   abs_err.mean(),
            'rmse':  np.sqrt((err ** 2).mean()),
            'bias':  err.mean(),
            'r2':    1 - ss_res / ss_tot if ss_tot > 0 else float('nan'),
            'p50':   np.percentile(abs_err, 50),
            'p75':   np.percentile(abs_err, 75),
            'p95':   np.percentile(abs_err, 95),
        })
    return pd.DataFrame(rows).set_index('label')


def main():
    df = pd.read_csv(ANALYTICS_PATH, index_col='sample_idx')
    summary = compute_summary(df)
    summary.to_csv(SUMMARY_PATH)
    print(f"saved: {SUMMARY_PATH}")
    print("\n=== error summary ===")
    print(summary.to_string(float_format='{:.4f}'.format))


if __name__ == '__main__':
    main()

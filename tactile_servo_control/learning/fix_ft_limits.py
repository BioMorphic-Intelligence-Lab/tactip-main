import os
import json
import pandas as pd

data_dir = "venv/lib/python3.12/site-packages/data/ur_tactip/surface_9d/data"

csv_path = os.path.join(data_dir, "targets.csv")
json_path = os.path.join(data_dir, "collect_params.json")

if not os.path.exists(csv_path) or not os.path.exists(json_path):
    print(f"Skipping {data_dir} — missing file.")
else:
    df = pd.read_csv(csv_path)
    ft_cols = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]

    # Compute limits
    ft_llims = df[ft_cols].min().round(4).tolist()
    ft_ulims = df[ft_cols].max().round(4).tolist()

    # Load and update JSON
    with open(json_path, "r") as f:
        collect_params = json.load(f)

    collect_params["ft_llims"] = ft_llims
    collect_params["ft_ulims"] = ft_ulims

    with open(json_path, "w") as f:
        json.dump(collect_params, f, indent=4)

    print(f"Updated {json_path}")
    print(f"ft_llims = {ft_llims}")
    print(f"ft_ulims = {ft_ulims}")

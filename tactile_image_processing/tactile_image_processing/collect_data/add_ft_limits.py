import os
import json
import pandas as pd

# This script adds the force/torque (FT) limits to the collect_params.json
# The limits are computed as the minimum and maximum measured values of the forces
# during the data collection sequence.

ft_cols = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]


def add_ft_limits(save_dir, train_dir=None, val_dir=None):
    # Write the FT limits in the data
    write_ft_limits_in_params(os.path.join(save_dir))

    if not train_dir == None and not val_dir == None:
        write_ft_limits_in_params(os.path.join(os.path.dirname(save_dir), train_dir))
        write_ft_limits_in_params(os.path.join(os.path.dirname(save_dir), val_dir))


def write_ft_limits_in_params(param_dir):   
    csv_path = os.path.join(param_dir, "targets.csv")
    json_path = os.path.join(param_dir, "collect_params.json")

    if not os.path.exists(csv_path):
        print(f"Skipping {param_dir} — no targets.csv")
        return

    if not os.path.exists(json_path):
        print(f"Skipping {param_dir} — no collect_params.json")
        return

    df = pd.read_csv(csv_path)

    # Compute limits
    ft_llims = df[ft_cols].min().round(4).tolist()
    ft_ulims = df[ft_cols].max().round(4).tolist()

    with open(json_path, "r") as f:
        collect_params = json.load(f)

    collect_params["ft_llims"] = ft_llims
    collect_params["ft_ulims"] = ft_ulims

    with open(json_path, "w") as f:
        json.dump(collect_params, f, indent=4)

    print(f"Updated {json_path}")
    print(f"ft_llims = {ft_llims}")
    print(f"ft_ulims = {ft_ulims}")

if __name__=="__main__":
    base_dir = "tactile_data_shear/data/ur_aerial-C2/surface_9d/data"
    add_ft_limits(
        base_dir,
        "train_data",
        "val_data"
    )

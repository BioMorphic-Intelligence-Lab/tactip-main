"""Entry point: python -m ship_emulation.run --help"""

import argparse
import logging
import sys

from ship_emulation.config import (
    EmulatorConfig,
    RobotConfig,
    SafetyConfig,
    WorkspaceLimits,
)
from ship_emulation.data_source import CsvFileSource, UdpSource
from ship_emulation.emulator import EmulationError, ShipEmulator
from ship_emulation.robot_interface import UR16Interface
from ship_emulation.safety import SafetyChecker


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python -m ship_emulation.run",
        description="Replay ship simulation data on a UR16 robot arm.",
    )
    p.add_argument("--source", choices=["csv", "udp"], required=True)
    p.add_argument("--csv-file", metavar="PATH", help="CSV file path (--source csv)")
    p.add_argument("--udp-port", type=int, default=5005, metavar="PORT")
    p.add_argument("--robot-ip", default="192.168.1.100", metavar="IP")
    p.add_argument("--x-range", type=float, nargs=2, default=[-500, 500], metavar=("MIN", "MAX"))
    p.add_argument("--y-range", type=float, nargs=2, default=[-500, 500], metavar=("MIN", "MAX"))
    p.add_argument("--z-range", type=float, nargs=2, default=[-500, 500], metavar=("MIN", "MAX"))
    p.add_argument("--dry-run", action="store_true", help="Validate data only, no robot connection")
    p.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return p


def main(argv=None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    if args.source == "csv" and not args.csv_file:
        parser.error("--csv-file is required when --source csv")

    workspace = WorkspaceLimits(
        x_min=args.x_range[0], x_max=args.x_range[1],
        y_min=args.y_range[0], y_max=args.y_range[1],
        z_min=args.z_range[0], z_max=args.z_range[1],
    )
    safety_cfg = SafetyConfig(workspace=workspace)
    robot_cfg = RobotConfig(ip=args.robot_ip)
    config = EmulatorConfig(robot=robot_cfg, safety=safety_cfg)
    safety = SafetyChecker(safety_cfg)

    source_ctx = (
        CsvFileSource(args.csv_file)
        if args.source == "csv"
        else UdpSource(port=args.udp_port)
    )

    if args.dry_run:
        print("Dry run — validating data only, no robot connection.")
        try:
            with source_ctx:
                for pose in source_ctx.poses():
                    safety.check(pose)
            print("Validation complete — no safety violations.")
        except Exception as e:
            print(f"Validation failed: {e}", file=sys.stderr)
            return 1
        return 0

    print(
        "\nWARNING: This will move the UR16 robot arm.\n"
        "Ensure the workspace is clear and the physical E-stop is within reach.\n"
        "Press ENTER to connect and start, or Ctrl+C to abort.\n"
    )
    try:
        input()
    except KeyboardInterrupt:
        print("\nAborted.")
        return 0

    try:
        with source_ctx, UR16Interface(config.robot) as interface:
            emulator = ShipEmulator(source_ctx, interface, safety, config)
            emulator.run()
    except EmulationError as e:
        print(f"Emulation error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())

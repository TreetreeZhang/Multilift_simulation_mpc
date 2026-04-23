"""Shared ROS2 helper functions for multilift nodes."""

import os

import yaml


def load_multilift_ros_defaults(package_share_directory=None):
    """Load ROS parameter defaults from multilift_params.yaml."""
    candidates = []
    if package_share_directory:
        candidates.append(
            os.path.join(package_share_directory, "config", "multilift_params.yaml")
        )

    candidates.extend([
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "config",
            "multilift_params.yaml",
        ),
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "multilift_params.yaml",
        ),
    ])

    config = {}
    for path in candidates:
        path = os.path.abspath(path)
        if os.path.exists(path):
            with open(path, "r") as stream:
                config = yaml.safe_load(stream) or {}
            break

    control = config.get("control", {})
    ros_defaults = config.get("ros_defaults", {})
    return {
        "uav_para": ros_defaults.get("uav_para", [1.0, 0.02, 0.02, 0.04, 6.0, 0.2]),
        "load_para": ros_defaults.get("load_para", [7.0, 1.0]),
        "cable_para": ros_defaults.get("cable_para", [1e9, 8e-6, 1e-2, 2.0]),
        "Jl": ros_defaults.get("Jl", [1.4, 1.4, 1.75]),
        "rg": ros_defaults.get("rg", [0.1, 0.1, -0.1]),
        "angle_t": control.get("angle_t", 0.3490658503988659),
        "altitude": control.get("altitude", 5.0),
        "dt_ctrl": control.get("dt_ctrl", 0.02),
        "dt_broadcast": control.get("dt_broadcast", 0.02),
        "horizon": control.get("horizon", 10),
        "horizon_loss": control.get("horizon_loss", 20),
        "gamma": control.get("gamma", 1e-4),
        "gamma2": control.get("gamma2", 1e-15),
        "drone_idx": ros_defaults.get("drone_idx", 0),
        "control_output_mode": ros_defaults.get("control_output_mode", "position"),
        "perf_report_window": ros_defaults.get("perf_report_window", 100),
        "force_start_mpc": ros_defaults.get("force_start_mpc", False),
        "trajectory_type": config.get("system", {}).get("trajectory_type", "fig8"),
    }

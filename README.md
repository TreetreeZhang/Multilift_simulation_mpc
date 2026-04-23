# Auto-Multilift_simulation

## Introduction

This branch provides parallel implementations of [Auto-Multilift](https://github.com/RCL-NUS/Auto-Multilift/), introducing two specific approaches—ROS topic and ROS service—to enable collaborative Model Predictive Control (MPC) solving and communication among multiple agents (multi-drones). These methods are designed to achieve faster parallel computation through enhanced cooperation.

> **Note:** This implementation is currently under active development and may contain unresolved issues.

## Prerequisites

### Auto-Multilift Dependencies

Before running the source code, please ensure that the following packages are installed with the specified versions:

- [CasADi](https://web.casadi.org/): 3.5.5  
- [ACADOS](https://github.com/acados/acados) (**stable version v0.3.5**; see [documentation](https://docs.acados.org/)).  
  **Important:** Do **not** use the latest version of ACADOS.
- [NumPy](https://numpy.org/): 1.23.0  
- [PyTorch](https://pytorch.org/): 1.12.0+cu116  
- [Matplotlib](https://matplotlib.org/): 3.3.0  
- [Python](https://www.python.org/): 3.9.12  
- [SciPy](https://scipy.org/): 1.8.1  
- [Pandas](https://pandas.pydata.org/): 1.4.2  
- [scikit-learn](https://scikit-learn.org/stable/whats_new/v1.0.html): 1.0.2  

After installing the above packages, you can validate your environment by running the following commands in the numerical simulation environment:
```
cd src/px4-offboard/px4_offboard/
python3 parallel_distributed_autotuning_evaluation_acados.py
```

### Building the ROS2 Workspace

Follow the [official PX4 ROS2 user guide](https://docs.px4.io/main/en/ros2/user_guide.html#installation-setup) to set up the ROS2 workspace.  
**Note:** During setup, ensure you install PX4 version **v1.14.3** using the following command:
```
git clone -b v1.14.3 https://github.com/PX4/PX4-Autopilot.git --recursive
```

### IsaacSim Installation

Use the [installation script](https://github.com/Temasek-Dynamics/SimulatorSetup) provided by the Temasek-Dynamics Laboratory to set up IsaacSim and PegasusSimulator.

After installation, edit the following configuration file:
```
cd path/to/SimulatorSetup/submodules/PegasusSimulator/extensions/pegasus.simulator/config/configs.yaml
```
Update the configuration as follows:
```
px4_default_airframe: none_iris
px4_dir: path/to/PX4-Autopilot   # v1.14.3, as installed above
```

## Run this project

After completing the installation and setup steps above, build and source the ROS2 workspace:

```bash
colcon build
source install/local_setup.bash
```

The recommended takeoff workflow is provided by:

```bash
./scripts/launch_takeoff.sh all
```

This opens three normal interactive terminal windows and runs the same three stages documented in `takeoff.md`:

1. Start the Micro XRCE-DDS Agent:

   ```bash
   ./scripts/launch_takeoff.sh agent
   ```

2. Start Isaac Sim / PX4 SITL:

   ```bash
   ./scripts/launch_takeoff.sh sim
   ```

3. Start the ROS2 MPC stack:

   ```bash
   ./scripts/launch_takeoff.sh ros
   ```

Before starting a full run, stale processes can be cleaned with:

```bash
./scripts/launch_takeoff.sh cleanup
```

The `all` command automatically runs `cleanup` before opening the three terminal windows. The script intentionally uses normal interactive bash shells so user-defined aliases, functions, and environment settings such as `ISAACSIM_PYTHON` remain available, matching the manual `takeoff.md` workflow as closely as possible.

If you prefer to run the system manually, open three terminals and run the commands above in order: `agent`, then `sim`, then `ros`.

## Code Structure

The repository is organized as a ROS2 workspace with simulation, message, PX4, and control packages:

```text
Auto-Multilift_simulation/
├── src/
│   ├── px4-offboard/          # Main ROS2 Python package: px4_offboard
│   ├── mpc_msgs/              # Custom MPC messages and services
│   ├── px4_msgs/              # PX4 message definitions
│   ├── px4_ros_com/           # PX4 ROS2 communication utilities
│   ├── px4_tf/                # PX4 frame transform utilities
│   └── sitl_sim/              # Isaac Sim / SITL simulation scripts
├── scripts/                   # Runtime helper scripts
├── data/                      # Trained models, reference trajectories, evaluation results
└── requirements.txt           # Python numerical and learning dependencies
```

The main `px4_offboard` package has been refactored into smaller engineering modules:

```text
src/px4-offboard/px4_offboard/
├── control/
│   ├── geometric_controller.py # Geometric controller
│   ├── mpc_solver.py           # MPC/acados solver and sensitivity utilities
│   └── l1_adaptive.py          # L1 adaptive compensation helper
├── dynamics/
│   ├── quadrotor.py            # Multilift dynamics model
│   └── payload.py              # Payload dynamics compatibility entry point
├── trajectory/
│   ├── reference_trajectory.py # Reference trajectory generation
│   └── trajectory_manager.py   # Precomputed trajectory manager
├── nodes/
│   ├── central_node.py         # Thin central-node compatibility entry point
│   └── quad_node.py            # Thin quad-node compatibility entry point
├── parallel/
│   ├── coordinator.py          # Parallel MPC compatibility entry point
│   └── worker.py               # Parallel MPC worker compatibility entry point
├── parallel_mpc/               # Shared-memory parallel MPC implementation
├── config/                     # YAML parameter loading utilities
└── utils/                      # Shared ROS/math helper utilities
```

Several legacy files remain as compatibility wrappers, including `Robust_Flight_MPC_acados.py`, `Dynamics.py`, `PrecomputedTrajectoryManager.py`, and `Reftraj.py`. Existing launch files and older scripts can still import these names, while new development should prefer the structured modules under `control/`, `dynamics/`, and `trajectory/`.

Runtime data have been moved out of the Python package:

```text
data/
├── trained_models/
├── reference_trajectories/
└── evaluation_results/
```

The ROS package setup still installs the required runtime resources into the package share directory, so existing resource lookup through `ament_index_python.get_package_share_directory('px4_offboard')` remains supported.

## References

- [Auto-Multilift](https://github.com/RCL-NUS/Auto-Multilift/)
- [PX4 Control Diagram](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html)
- [px4_offboard (Offboard Demo Source)](https://github.com/Jaeyoung-Lim/px4-offboard)
- [ROS 2 Offboard Control Example](https://docs.px4.io/main/en/ros2/offboard_control.html)

#!/usr/bin/env python3
"""
Parameter Management System
Provides YAML-based configuration with dataclass encapsulation
"""

import yaml
from dataclasses import dataclass, field
from typing import List, Optional
from pathlib import Path
import numpy as np

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


@dataclass
class UAVParameters:
    """UAV physical parameters"""
    mass: float = 1.0
    Ix: float = 0.02
    Iy: float = 0.02
    Iz: float = 0.04
    wing_length: float = 0.2
    num_uavs: int = 6

    def to_array(self) -> np.ndarray:
        """Convert to array format used by dynamics model"""
        return np.array([self.mass, self.Ix, self.Iy, self.Iz, self.num_uavs, self.wing_length])


@dataclass
class PayloadParameters:
    """Payload physical parameters"""
    mass: float = 7.5
    radius: float = 0.3
    height: float = 0.05
    Jx: float = 1.4
    Jy: float = 1.4
    Jz: float = 1.75
    center_of_mass: List[float] = field(default_factory=lambda: [0.1, 0.1, -0.1])

    def to_mass_array(self) -> np.ndarray:
        """Convert mass to array format"""
        return np.array([self.mass, self.radius, self.height])

    def to_inertia_array(self) -> np.ndarray:
        """Convert inertia to array format"""
        return np.array([self.Jx, self.Jy, self.Jz])

    def to_com_array(self) -> np.ndarray:
        """Convert center of mass to array format"""
        return np.array(self.center_of_mass).reshape(-1, 1)


@dataclass
class CableParameters:
    """Cable physical parameters"""
    youngs_modulus: float = 5e3
    cross_section_area: float = 1e-2
    damping_coefficient: float = 2.0
    rest_length: float = 2.0

    def to_array(self) -> np.ndarray:
        """Convert to array format"""
        return np.array([
            self.youngs_modulus,
            self.cross_section_area,
            self.damping_coefficient,
            self.rest_length
        ])


@dataclass
class ControlParameters:
    """Control system parameters"""
    dt_ctrl: float = 0.05
    dt_broadcast: float = 0.02
    horizon: int = 10
    angle_t: float = np.pi / 9


@dataclass
class TrajectoryParameters:
    """Trajectory generation parameters"""
    T_total: float = 20.0
    dt_ctrl: float = 0.05
    nq: int = 6
    type: str = "fig8"


@dataclass
class PrecomputedTrajectoryParameters:
    """Precomputed trajectory parameters"""
    data_path: str = "precomputed_trajectories"
    horizon: int = 10
    dt_ctrl: float = 0.05
    auto_generate: bool = True


@dataclass
class ParallelMPCParameters:
    """Parallel MPC parameters"""
    enabled: bool = True
    num_workers: int = 6
    deadline_ms: int = 10
    gamma: float = 1e-2
    gamma2: float = 1e-3
    quad_weightings: List[float] = field(default_factory=lambda: [1.0] * 28)
    load_weightings: List[float] = field(default_factory=lambda: [1.0] * 30)


@dataclass
class SystemParameters:
    """Complete system parameters container"""
    uav: UAVParameters = field(default_factory=UAVParameters)
    payload: PayloadParameters = field(default_factory=PayloadParameters)
    cable: CableParameters = field(default_factory=CableParameters)
    control: ControlParameters = field(default_factory=ControlParameters)
    trajectory: TrajectoryParameters = field(default_factory=TrajectoryParameters)
    precomputed_trajectory: PrecomputedTrajectoryParameters = field(
        default_factory=PrecomputedTrajectoryParameters
    )
    parallel_mpc: ParallelMPCParameters = field(default_factory=ParallelMPCParameters)

    num_drones: int = 6
    trajectory_type: str = "fig8"
    auto_generate_trajectories: bool = True


class ParameterManager:
    """Manages loading and accessing system parameters from YAML files"""

    @staticmethod
    def _default_config_candidates() -> List[Path]:
        current_dir = Path(__file__).parent
        candidates = [current_dir / "multilift_params.yaml"]

        if get_package_share_directory is not None:
            try:
                share_dir = Path(get_package_share_directory("px4_offboard"))
                candidates.append(share_dir / "config" / "multilift_params.yaml")
            except Exception:
                pass

        return candidates

    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize parameter manager

        Args:
            config_path: Path to YAML configuration file. If None, uses default location.
        """
        if config_path is None:
            candidates = self._default_config_candidates()
            config_path = next((path for path in candidates if path.exists()), candidates[0])

        self.config_path = Path(config_path)
        self.params = self._load_parameters()

    def _load_parameters(self) -> SystemParameters:
        """Load parameters from YAML file"""
        if not self.config_path.exists():
            raise FileNotFoundError(
                f"Configuration file not found: {self.config_path}\n"
                f"Using default parameters."
            )

        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)

        uav_config = config.get('uav', {})
        uav_inertia = uav_config.get('inertia', {})

        payload_config = config.get('payload', {})
        payload_inertia = payload_config.get('inertia', {})
        payload_com = payload_config.get('center_of_mass', {})

        return SystemParameters(
            uav=UAVParameters(
                mass=uav_config.get('mass', 1.0),
                Ix=uav_inertia.get('Ix', 0.02),
                Iy=uav_inertia.get('Iy', 0.02),
                Iz=uav_inertia.get('Iz', 0.04),
                wing_length=uav_config.get('wing_length', 0.2),
                num_uavs=uav_config.get('num_uavs', 6)
            ),
            payload=PayloadParameters(
                mass=payload_config.get('mass', 7.5),
                radius=payload_config.get('radius', 0.3),
                height=payload_config.get('height', 0.05),
                Jx=payload_inertia.get('Jx', 1.4),
                Jy=payload_inertia.get('Jy', 1.4),
                Jz=payload_inertia.get('Jz', 1.75),
                center_of_mass=[
                    payload_com.get('x', 0.1) if isinstance(payload_com, dict) else payload_com[0] if isinstance(payload_com, list) and len(payload_com) > 0 else 0.1,
                    payload_com.get('y', 0.1) if isinstance(payload_com, dict) else payload_com[1] if isinstance(payload_com, list) and len(payload_com) > 1 else 0.1,
                    payload_com.get('z', -0.1) if isinstance(payload_com, dict) else payload_com[2] if isinstance(payload_com, list) and len(payload_com) > 2 else -0.1
                ]
            ),
            cable=CableParameters(**config.get('cable', {})),
            control=ControlParameters(**config.get('control', {})),
            trajectory=TrajectoryParameters(**config.get('trajectory', {})),
            precomputed_trajectory=PrecomputedTrajectoryParameters(
                **config.get('precomputed_trajectory', {})
            ),
            parallel_mpc=ParallelMPCParameters(
                **config.get('parallel_mpc', {})
            ),
            num_drones=config.get('system', {}).get('num_drones', 6),
            trajectory_type=config.get('system', {}).get('trajectory_type', 'fig8'),
            auto_generate_trajectories=config.get('system', {}).get(
                'auto_generate_trajectories', True
            )
        )

    def get_uav_params(self) -> np.ndarray:
        """Get UAV parameters as array (for dynamics model)"""
        return self.params.uav.to_array()

    def get_payload_mass_params(self) -> np.ndarray:
        """Get payload mass parameters as array"""
        return self.params.payload.to_mass_array()

    def get_payload_inertia(self) -> np.ndarray:
        """Get payload inertia as array"""
        return self.params.payload.to_inertia_array()

    def get_cable_params(self) -> np.ndarray:
        """Get cable parameters as array"""
        return self.params.cable.to_array()

    def save_config(self, output_path: Optional[str] = None):
        """Save current parameters to YAML file"""
        if output_path is None:
            output_path = self.config_path

        config_dict = {
            'system': {
                'num_drones': self.params.num_drones,
                'trajectory_type': self.params.trajectory_type,
                'auto_generate_trajectories': self.params.auto_generate_trajectories
            },
            'uav': {
                'mass': self.params.uav.mass,
                'inertia': {
                    'Ix': self.params.uav.Ix,
                    'Iy': self.params.uav.Iy,
                    'Iz': self.params.uav.Iz
                },
                'wing_length': self.params.uav.wing_length,
                'num_uavs': self.params.uav.num_uavs
            },
            'payload': {
                'mass': self.params.payload.mass,
                'radius': self.params.payload.radius,
                'height': self.params.payload.height,
                'inertia': {
                    'Jx': self.params.payload.Jx,
                    'Jy': self.params.payload.Jy,
                    'Jz': self.params.payload.Jz
                },
                'center_of_mass': {
                    'x': self.params.payload.center_of_mass[0],
                    'y': self.params.payload.center_of_mass[1],
                    'z': self.params.payload.center_of_mass[2]
                }
            },
            'cable': {
                'youngs_modulus': self.params.cable.youngs_modulus,
                'cross_section_area': self.params.cable.cross_section_area,
                'damping_coefficient': self.params.cable.damping_coefficient,
                'rest_length': self.params.cable.rest_length
            },
            'control': {
                'dt_ctrl': self.params.control.dt_ctrl,
                'dt_broadcast': self.params.control.dt_broadcast,
                'horizon': self.params.control.horizon,
                'angle_t': self.params.control.angle_t
            },
            'trajectory': {
                'T_total': self.params.trajectory.T_total,
                'dt_ctrl': self.params.trajectory.dt_ctrl,
                'nq': self.params.trajectory.nq,
                'type': self.params.trajectory.type
            },
            'precomputed_trajectory': {
                'data_path': self.params.precomputed_trajectory.data_path,
                'horizon': self.params.precomputed_trajectory.horizon,
                'dt_ctrl': self.params.precomputed_trajectory.dt_ctrl,
                'auto_generate': self.params.precomputed_trajectory.auto_generate
            },
            'parallel_mpc': {
                'enabled': self.params.parallel_mpc.enabled,
                'num_workers': self.params.parallel_mpc.num_workers,
                'deadline_ms': self.params.parallel_mpc.deadline_ms,
                'gamma': self.params.parallel_mpc.gamma,
                'gamma2': self.params.parallel_mpc.gamma2,
                'quad_weightings': self.params.parallel_mpc.quad_weightings,
                'load_weightings': self.params.parallel_mpc.load_weightings
            }
        }

        with open(output_path, 'w') as f:
            yaml.dump(config_dict, f, default_flow_style=False, sort_keys=False)


if __name__ == "__main__":
    param_manager = ParameterManager()

    print("=== System Parameters ===")
    print(f"Number of drones: {param_manager.params.num_drones}")
    print(f"Trajectory type: {param_manager.params.trajectory_type}")
    print(f"\nUAV Parameters: {param_manager.get_uav_params()}")
    print(f"Payload Mass: {param_manager.get_payload_mass_params()}")
    print(f"Cable Parameters: {param_manager.get_cable_params()}")
    print(f"Control dt: {param_manager.params.control.dt_ctrl}s")
    print(f"MPC Horizon: {param_manager.params.control.horizon}")

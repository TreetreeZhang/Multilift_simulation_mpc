#!/usr/bin/env python3
"""
预计算轨迹生成器
将所有轨迹点预先计算并存储，实现零延迟轨迹查找
"""

import numpy as np
import os
import time
import json
from typing import Dict, Tuple
from pathlib import Path

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

try:
    from .dynamics import multilifting
    from .config.parameter_manager import ParameterManager
    from .control import Controller
except ImportError:
    from dynamics import multilifting
    from config.parameter_manager import ParameterManager
    from control import Controller


class PrecomputedTrajectoryGenerator:
    """预计算轨迹生成器"""

    @staticmethod
    def _candidate_resource_paths(resource_name: str) -> list[str]:
        package_dir = Path(__file__).resolve().parent
        candidates = [
            package_dir / resource_name,
            package_dir.parent / resource_name,
            package_dir.parent.parent / "share" / "px4_offboard" / resource_name,
            Path.cwd() / resource_name,
        ]

        if get_package_share_directory is not None:
            try:
                share_dir = Path(get_package_share_directory("px4_offboard"))
                candidates.append(share_dir / resource_name)
            except Exception:
                pass

        unique_candidates: list[str] = []
        seen = set()
        for path in candidates:
            resolved = str(path)
            if resolved not in seen:
                seen.add(resolved)
                unique_candidates.append(resolved)
        return unique_candidates

    def __init__(self,
                 T_total: float | None = None,
                 dt_ctrl: float | None = None,
                 nq: int | None = None,
                 trajectory_type: str | None = None,
                 config_path: str | None = None,
                 uav_para: np.ndarray | None = None,
                 load_para: np.ndarray | None = None,
                 cable_para: np.ndarray | None = None,
                 angle_t: float | None = None):
        """
        初始化预计算轨迹生成器

        Args:
            T_total: 总任务时间（秒）
            dt_ctrl: 控制周期（秒）
            nq: 无人机数量
            trajectory_type: 轨迹类型 ('fig8', 'circle', 'hover')
            config_path: 参数配置路径
        """
        params = ParameterManager(config_path).params
        self.T_total = float(T_total) if T_total is not None else float(params.trajectory.T_total)
        self.dt_ctrl = float(dt_ctrl) if dt_ctrl is not None else float(params.control.dt_ctrl)
        self.nq = int(nq) if nq is not None else int(params.uav.num_uavs)
        self.trajectory_type = trajectory_type or params.trajectory.type
        self._params = params
        self._uav_para_override = uav_para
        self._load_para_override = load_para
        self._cable_para_override = cable_para
        self._angle_t_override = angle_t

        # 计算时间点
        self.total_time_points = int(self.T_total / self.dt_ctrl) + 1
        self.time_vector = np.arange(0, self.T_total + self.dt_ctrl, self.dt_ctrl)

        # 初始化动力学模型
        self._init_dynamics_model()

        # 轨迹存储结构
        self.trajectory_cache: Dict[str, Dict[str, np.ndarray]] = {}
        self.angle_cache = None

        print("预计算配置:")
        print(f"  总时间: {self.T_total}s")
        print(f"  控制周期: {self.dt_ctrl}s")
        print(f"  无人机数量: {self.nq}")
        print(f"  时间点数量: {self.total_time_points}")
        print(f"  轨迹类型: {self.trajectory_type}")

    def _init_dynamics_model(self) -> None:
        """初始化动力学模型"""
        uav_para = np.array(self._uav_para_override, dtype=float) if self._uav_para_override is not None else self._params.uav.to_array().astype(float)
        uav_para[4] = self.nq
        if self._load_para_override is not None:
            load_para = np.array(self._load_para_override, dtype=float)[:2]
        else:
            load_para = self._params.payload.to_mass_array().astype(float)[:2]
        cable_para = np.array(self._cable_para_override, dtype=float) if self._cable_para_override is not None else self._params.cable.to_array().astype(float)

        self.stm = multilifting(uav_para, load_para, cable_para, self.dt_ctrl)
        self.stm.model()
        self.geo_ctrl = Controller(uav_para, self.dt_ctrl)

        # 加载轨迹系数
        self._load_trajectory_coefficients()

    def _load_trajectory_coefficients(self) -> None:
        """加载轨迹系数"""
        if self.trajectory_type == 'fig8':
            self.Coeffx = np.zeros((8, 8))
            self.Coeffy = np.zeros((8, 8))
            self.Coeffz = np.zeros((8, 8))

            possible_paths = self._candidate_resource_paths('Reference_traj_fig8')

            coeff_dir = None
            for path in possible_paths:
                if os.path.exists(path) and os.path.exists(os.path.join(path, 'coeffxl_1.npy')):
                    coeff_dir = path
                    break

            if coeff_dir is None:
                raise FileNotFoundError(f"找不到轨迹系数文件，尝试的路径: {possible_paths}")

            for k in range(8):
                self.Coeffx[k, :] = np.load(os.path.join(coeff_dir, f'coeffxl_{k+1}.npy')).flatten()
                self.Coeffy[k, :] = np.load(os.path.join(coeff_dir, f'coeffyl_{k+1}.npy')).flatten()
                self.Coeffz[k, :] = np.load(os.path.join(coeff_dir, f'coeffzl_{k+1}.npy')).flatten()
        elif self.trajectory_type == 'circle':
            possible_paths = self._candidate_resource_paths('Reference_traj_circle')

            coeff_file = None
            for path in possible_paths:
                coeff_path = os.path.join(path, 'coeffa.npy')
                if os.path.exists(coeff_path):
                    coeff_file = coeff_path
                    break

            if coeff_file is None:
                raise FileNotFoundError(f"找不到circle轨迹系数文件coeffa.npy，尝试的路径: {possible_paths}")

            self.coeffa = np.load(coeff_file)
            if self.coeffa.shape[0] > 1:
                self.coeffa = self.coeffa[0, :]
            else:
                self.coeffa = self.coeffa.flatten()
        else:
            raise ValueError(f"不支持的轨迹类型: {self.trajectory_type}")

    def generate_all_trajectories(self, save_path: str = "precomputed_trajectories") -> None:
        """生成所有轨迹并保存"""
        print("开始预计算所有轨迹...")
        start_time = time.time()

        current_dir = os.path.dirname(os.path.abspath(__file__))
        full_save_path = save_path if os.path.isabs(save_path) else os.path.join(current_dir, save_path)
        os.makedirs(full_save_path, exist_ok=True)

        self._compute_angle_trajectory()

        for quad_id in range(self.nq):
            print(f"计算无人机 {quad_id + 1}/{self.nq} 的轨迹...")
            self._compute_quadrotor_trajectory(quad_id)

        print("计算载荷轨迹...")
        self._compute_payload_trajectory()

        self._save_trajectories(full_save_path)

        end_time = time.time()
        print(f"预计算完成！耗时: {end_time - start_time:.2f}秒")
        print(f"轨迹数据已保存到: {full_save_path}")

    def _compute_angle_trajectory(self) -> None:
        """计算角度轨迹"""
        self.angle_cache = np.zeros(self.total_time_points, dtype=float)
        for i, t in enumerate(self.time_vector):
            if self._angle_t_override is not None:
                self.angle_cache[i] = float(self._angle_t_override)
            else:
                self.angle_cache[i] = float(self._params.control.angle_t)

    def _compute_quadrotor_trajectory(self, quad_id: int) -> None:
        """计算单架无人机的完整轨迹"""
        quad_trajectory = {
            'position': np.zeros((self.total_time_points, 3)),
            'velocity': np.zeros((self.total_time_points, 3)),
            'acceleration': np.zeros((self.total_time_points, 3)),
            'reference_state': np.zeros((self.total_time_points, 13)),
            'reference_control': np.zeros((self.total_time_points, 4))
        }

        for i, t in enumerate(self.time_vector):
            angle_t = self.angle_cache[i]

            if self.trajectory_type == 'fig8':
                ref_p, ref_v, ref_a = self.stm.minisnap_quadrotor_fig8(
                    self.Coeffx, self.Coeffy, self.Coeffz, t, angle_t, quad_id)
            elif self.trajectory_type == 'circle':
                ref_p, ref_v, ref_a = self.stm.new_circle_quadrotor(
                    self.coeffa, t, angle_t, quad_id)
            else:
                raise ValueError(f"不支持的轨迹类型: {self.trajectory_type}")

            if self.trajectory_type == 'fig8':
                ref_pl, ref_vl, ref_al = self.stm.minisnap_load_fig8(
                    self.Coeffx, self.Coeffy, self.Coeffz, t)
            else:
                ref_pl, ref_vl, ref_al = self.stm.new_circle_load(
                    self.coeffa, t)

            qd, wd, f_ref, fl_ref, M_ref = self.geo_ctrl.system_ref(
                ref_a, self._params.payload.mass, ref_al
            )

            quad_trajectory['position'][i, :] = ref_p.flatten()
            quad_trajectory['velocity'][i, :] = ref_v.flatten()
            quad_trajectory['acceleration'][i, :] = ref_a.flatten()

            ref_xi = np.vstack((ref_p, ref_v, qd, wd))
            ref_ui = np.vstack((f_ref, M_ref))

            if ref_xi.shape[0] >= 13:
                quad_trajectory['reference_state'][i, :] = ref_xi[:13].flatten()
            else:
                padded_xi = np.zeros((13, 1))
                padded_xi[:ref_xi.shape[0]] = ref_xi
                quad_trajectory['reference_state'][i, :] = padded_xi.flatten()

            quad_trajectory['reference_control'][i, :] = ref_ui[:4].flatten()

        self.trajectory_cache[f'quad_{quad_id}'] = quad_trajectory

    def _compute_payload_trajectory(self) -> None:
        """计算载荷的完整轨迹"""
        payload_trajectory = {
            'position': np.zeros((self.total_time_points, 3)),
            'velocity': np.zeros((self.total_time_points, 3)),
            'acceleration': np.zeros((self.total_time_points, 3)),
            'reference_state': np.zeros((self.total_time_points, 13)),
            'reference_control': np.zeros((self.total_time_points, self.nq))
        }

        for i, t in enumerate(self.time_vector):
            if self.trajectory_type == 'fig8':
                ref_pl, ref_vl, ref_al = self.stm.minisnap_load_fig8(
                    self.Coeffx, self.Coeffy, self.Coeffz, t)
                ref_p, ref_v, ref_a = self.stm.minisnap_quadrotor_fig8(
                    self.Coeffx, self.Coeffy, self.Coeffz, t, self.angle_cache[i], 0)
            elif self.trajectory_type == 'circle':
                ref_pl, ref_vl, ref_al = self.stm.new_circle_load(
                    self.coeffa, t)
                ref_p, ref_v, ref_a = self.stm.new_circle_quadrotor(
                    self.coeffa, t, self.angle_cache[i], 0)
            else:
                raise ValueError(f"不支持的轨迹类型: {self.trajectory_type}")

            qld = np.array([1, 0, 0, 0])
            wld = np.zeros(3)
            ref_xl = np.concatenate([ref_pl.flatten(), ref_vl.flatten(), qld, wld])
            _, _, _, fl_ref, _ = self.geo_ctrl.system_ref(
                ref_a, self._params.payload.mass, ref_al
            )
            ref_ul = (fl_ref / self.nq * np.ones((self.nq, 1))).flatten()

            payload_trajectory['position'][i, :] = ref_pl.flatten()
            payload_trajectory['velocity'][i, :] = ref_vl.flatten()
            payload_trajectory['acceleration'][i, :] = ref_al.flatten()
            payload_trajectory['reference_state'][i, :] = ref_xl
            payload_trajectory['reference_control'][i, :] = ref_ul

        self.trajectory_cache['payload'] = payload_trajectory

    def _save_trajectories(self, save_path: str) -> None:
        """保存轨迹数据（使用JSON+NumPy格式）"""
        for i, trajectory in self.trajectory_cache.items():
            trajectory_file = os.path.join(save_path, f'trajectory_{i}.npy')
            np.save(trajectory_file, trajectory)

        if self.angle_cache is not None:
            angle_file = os.path.join(save_path, 'angle_cache.npy')
            np.save(angle_file, self.angle_cache)

        config_angle = self._angle_t_override if self._angle_t_override is not None else self._params.control.angle_t
        config = {
            'T_total': self.T_total,
            'dt_ctrl': self.dt_ctrl,
            'nq': self.nq,
            'trajectory_type': self.trajectory_type,
            'total_time_points': self.total_time_points,
            'angle_t': float(config_angle),
        }

        config_file = os.path.join(save_path, 'config.json')
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=2)

        np.save(os.path.join(save_path, 'time_vector.npy'), self.time_vector)

        print("轨迹数据保存完成:")
        print(f"  - trajectory_*.npy: {len(self.trajectory_cache)} 个轨迹文件")
        print("  - angle_cache.npy: 角度数据")
        print("  - config.json: 配置信息")
        print("  - time_vector.npy: 时间向量")


class PrecomputedTrajectoryLookup:
    """预计算轨迹查找器"""

    def __init__(self, data_path: str = "precomputed_trajectories"):
        """初始化轨迹查找器"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.data_path = data_path if os.path.isabs(data_path) else os.path.join(current_dir, data_path)
        self.trajectory_cache = {}
        self.angle_cache = None
        self.config = {}
        self.time_vector = None

        self._load_precomputed_data()

    def _load_precomputed_data(self) -> None:
        """加载预计算的轨迹数据"""
        print(f"从 {self.data_path} 加载预计算轨迹...")

        config_file = os.path.join(self.data_path, 'config.json')
        if not os.path.exists(config_file):
            raise FileNotFoundError(f"预计算数据不存在: {self.data_path}")

        with open(config_file, 'r') as f:
            self.config = json.load(f)

        self.time_vector = np.load(os.path.join(self.data_path, 'time_vector.npy'))
        self.trajectory_cache = {}
        for filename in os.listdir(self.data_path):
            if not filename.startswith("trajectory_") or not filename.endswith(".npy"):
                continue
            key = filename[len("trajectory_"):-len(".npy")]
            self.trajectory_cache[key] = np.load(os.path.join(self.data_path, filename), allow_pickle=True).item()

        angle_file = os.path.join(self.data_path, 'angle_cache.npy')
        if os.path.exists(angle_file):
            self.angle_cache = np.load(angle_file)
        else:
            self.angle_cache = None

        print("预计算数据加载成功:")
        print(f"  时间范围: 0 - {self.config['T_total']}s")
        print(f"  控制周期: {self.config['dt_ctrl']}s")
        print(f"  无人机数量: {self.config['nq']}")
        print(f"  轨迹类型: {self.config['trajectory_type']}")

    def get_trajectory_point(self, time: float, quad_id: int | None = None,
                             return_type: str = 'full') -> Dict:
        """获取指定时间的轨迹点"""
        time_idx = np.argmin(np.abs(self.time_vector - time))
        trajectory = self.trajectory_cache[f'quad_{quad_id}'] if quad_id is not None else self.trajectory_cache['payload']

        if return_type == 'position':
            return {'position': trajectory['position'][time_idx, :]}
        if return_type == 'velocity':
            return {'velocity': trajectory['velocity'][time_idx, :]}
        if return_type == 'acceleration':
            return {'acceleration': trajectory['acceleration'][time_idx, :]}
        return {
            'position': trajectory['position'][time_idx, :],
            'velocity': trajectory['velocity'][time_idx, :],
            'acceleration': trajectory['acceleration'][time_idx, :],
            'reference_state': trajectory['reference_state'][time_idx, :],
            'reference_control': trajectory['reference_control'][time_idx, :]
        }

    def get_trajectory_horizon(self, start_time: float, horizon: int = 10,
                               quad_id: int | None = None) -> Dict:
        """获取预测时域的轨迹"""
        start_idx = np.argmin(np.abs(self.time_vector - start_time))
        if start_idx + horizon >= len(self.time_vector):
            raise ValueError("预测时域超出预计算范围")

        trajectory = self.trajectory_cache[f'quad_{quad_id}'] if quad_id is not None else self.trajectory_cache['payload']
        horizon_data = {}
        for key in trajectory.keys():
            horizon_data[key] = trajectory[key][start_idx:start_idx + horizon + 1, :]

        return horizon_data

    def get_angle(self, time: float) -> float:
        """获取指定时间的角度"""
        if self.angle_cache is None or len(self.angle_cache) == 0:
            return 0.0
        time_idx = np.argmin(np.abs(self.time_vector - time))
        return float(self.angle_cache[time_idx])


def main() -> None:
    """主函数：演示预计算和查找过程"""
    print("=== 预计算轨迹生成器演示 ===\n")

    generator = PrecomputedTrajectoryGenerator(
        T_total=20.0,
        dt_ctrl=0.02,
        nq=6,
        trajectory_type='fig8'
    )

    generator.generate_all_trajectories("precomputed_trajectories")

    lookup = PrecomputedTrajectoryLookup("precomputed_trajectories")
    test_times = [0.0, 5.0, 10.0, 15.0, 19.0]
    test_quads = [0, 1, 2, 3, 4, 5]

    print("单点查找测试:")
    start_time = time.time()
    for t in test_times:
        for q in test_quads:
            lookup.get_trajectory_point(t, q)
    end_time = time.time()
    total_lookups = len(test_times) * len(test_quads)
    avg_time = (end_time - start_time) / total_lookups * 1000
    print(f"  平均查找时间: {avg_time:.3f} ms")


if __name__ == "__main__":
    main()
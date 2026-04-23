#!/usr/bin/env python3
"""
预计算轨迹管理器
集成到现有MPC系统中，提供零延迟轨迹查找

作者: AI Assistant  
日期: 2024
"""

import numpy as np
import os
import pickle
import json
import time
from typing import Dict, List, Tuple, Optional

try:
    from .config.parameter_manager import ParameterManager
except Exception:
    try:
        from config.parameter_manager import ParameterManager
    except Exception:
        ParameterManager = None

class PrecomputedTrajectoryManager:
    """预计算轨迹管理器 - 与现有MPC系统集成"""
    
    def __init__(self, 
                 data_path: str = "precomputed_trajectories",
                 horizon: int = 10,
                 dt_ctrl: float = 0.02,
                 auto_generate: bool = True,
                 uav_para: Optional[np.ndarray] = None,
                 load_para: Optional[np.ndarray] = None,
                 cable_para: Optional[np.ndarray] = None,
                 angle_t: Optional[float] = None,
                 nq: Optional[int] = None):
        """
        初始化预计算轨迹管理器
        
        Args:
            data_path: 预计算数据路径
            horizon: MPC预测时域
            dt_ctrl: 控制周期
            auto_generate: 是否自动生成预计算数据
        """
        self.data_path = data_path
        self.horizon = horizon
        self.dt_ctrl = dt_ctrl
        self.auto_generate = auto_generate
        self._uav_para_override = uav_para
        self._load_para_override = load_para
        self._cable_para_override = cable_para
        self._angle_t_override = angle_t
        self._nq_override = nq
        self._params = ParameterManager().params if ParameterManager else None
        
        # 轨迹缓存
        self.trajectory_cache = {}
        self.angle_cache = {}
        self.config = {}
        self.time_vector = None
        self.config_angle_t = None
        self._angle_warned = False
        
        # 性能统计
        self.lookup_count = 0
        self.total_lookup_time = 0.0
        
        # 确保数据路径是绝对路径
        if not os.path.isabs(self.data_path):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.data_path = os.path.join(current_dir, self.data_path)
        
        # 加载预计算数据，如果不存在且启用自动生成则生成
        self._load_or_generate_data()
    
    def _load_or_generate_data(self):
        """加载预计算数据，如果不存在则自动生成"""
        # 检查数据是否存在
        config_file = os.path.join(self.data_path, 'config.json')
        
        if os.path.exists(config_file):
            # 数据存在，直接加载
            self._load_precomputed_data()
        elif self.auto_generate:
            # 数据不存在且启用自动生成，则生成
            print("🚀 预计算数据不存在，开始自动生成...")
            self._auto_generate_trajectories()
        else:
            # 数据不存在且未启用自动生成，抛出异常
            raise FileNotFoundError(f"预计算数据不存在于 {self.data_path}，请启用auto_generate=True或手动生成数据")
    
    def _auto_generate_trajectories(self):
        """自动生成预计算轨迹"""
        try:
            # 尝试导入预计算生成器
            try:
                from px4_offboard.precompute_trajectory_generator import PrecomputedTrajectoryGenerator
            except ImportError:
                # 如果ROS2包中没有，尝试直接导入
                import sys
                import os
                current_dir = os.path.dirname(os.path.abspath(__file__))
                sys.path.append(current_dir)
                from precompute_trajectory_generator import PrecomputedTrajectoryGenerator
            
            # 创建生成器
            t_total = self._params.trajectory.T_total if self._params else 20.0
            num_uavs = self._nq_override if self._nq_override is not None else (self._params.uav.num_uavs if self._params else 6)
            traj_type = self._params.trajectory.type if self._params else 'fig8'
            generator = PrecomputedTrajectoryGenerator(
                T_total=t_total,
                dt_ctrl=self.dt_ctrl,
                nq=num_uavs,
                trajectory_type=traj_type,
                uav_para=self._uav_para_override,
                load_para=self._load_para_override,
                cable_para=self._cable_para_override,
                angle_t=self._angle_t_override,
            )
            
            # 生成所有轨迹
            print("⏳ 正在生成预计算轨迹，请稍候...")
            generator.generate_all_trajectories(self.data_path)
            print(f"✅ 轨迹生成完成！")
            
            # 生成完成后加载数据
            self._load_precomputed_data()
            
        except ImportError as e:
            raise ImportError(f"导入预计算生成器失败: {e}")
        except Exception as e:
            raise RuntimeError(f"自动生成轨迹失败: {e}")
    
    def _load_precomputed_data(self):
        """加载预计算的轨迹数据（使用JSON+NumPy格式）"""
        try:
            print(f"📂 加载预计算轨迹数据: {self.data_path}")
            
            # 加载配置（JSON格式）
            config_file = os.path.join(self.data_path, 'config.json')
            with open(config_file, 'r') as f:
                self.config = json.load(f)
            self.config_angle_t = self.config.get("angle_t")
            
            # 加载时间向量（NumPy原生格式）
            self.time_vector = np.load(os.path.join(self.data_path, 'time_vector.npy'))
            
            # 加载轨迹缓存（NumPy原生格式）
            self.trajectory_cache = {}
            for filename in os.listdir(self.data_path):
                if not filename.startswith("trajectory_") or not filename.endswith(".npy"):
                    continue
                key = filename[len("trajectory_"):-len(".npy")]
                cache_file = os.path.join(self.data_path, filename)
                self.trajectory_cache[key] = np.load(cache_file, allow_pickle=True).item()
            
            # 加载角度缓存（NumPy原生格式，允许pickle）
            angle_file = os.path.join(self.data_path, 'angle_cache.npy')
            if os.path.exists(angle_file):
                self.angle_cache = np.load(angle_file, allow_pickle=True)
            else:
                self.angle_cache = None
            
            print(f"✅ 预计算数据加载成功:")
            print(f"  时间范围: 0 - {self.config['T_total']}s")
            print(f"  控制周期: {self.config['dt_ctrl']}s")
            print(f"  无人机数量: {self.config['nq']}")
            print(f"  轨迹类型: {self.config['trajectory_type']}")
            
        except FileNotFoundError:
            print(f"❌ 预计算数据文件损坏: {self.data_path}")
            if self.auto_generate:
                print("🔄 尝试重新生成...")
                self._auto_generate_trajectories()
            else:
                raise FileNotFoundError(f"预计算数据文件损坏，请重新生成数据")
    
    def get_time_index(self, time: float) -> int:
        """获取最接近的时间索引"""
        return np.argmin(np.abs(self.time_vector - time))
    
    def get_angle(self, time: float) -> float:
        """获取指定时间的角度"""
        if len(self.angle_cache) == 0:
            return 0.0
        
        time_idx = self.get_time_index(time)
        return self.angle_cache[self.time_vector[time_idx]]
    
    def get_quadrotor_trajectory_point(self, time: float, quad_id: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        获取单架无人机的轨迹点
        
        Args:
            time: 时间点
            quad_id: 无人机ID
            
        Returns:
            (position, velocity, acceleration)
        """
        import time as time_module
        start_time = time_module.time()
        
        if f'quad_{quad_id}' not in self.trajectory_cache:
            # 如果缓存中不存在，返回零值
            return (np.zeros(3), np.zeros(3), np.zeros(3))
        
        time_idx = self.get_time_index(time)
        trajectory = self.trajectory_cache[f'quad_{quad_id}']
        
        position = trajectory['position'][time_idx, :]
        velocity = trajectory['velocity'][time_idx, :]
        acceleration = trajectory['acceleration'][time_idx, :]
        
        # 性能统计
        self.lookup_count += 1
        self.total_lookup_time += time_module.time() - start_time
        
        return (position, velocity, acceleration)
    
    def get_payload_trajectory_point(self, time: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        获取载荷的轨迹点
        
        Args:
            time: 时间点
            
        Returns:
            (position, velocity, acceleration)
        """
        if 'payload' not in self.trajectory_cache:
            return (np.zeros(3), np.zeros(3), np.zeros(3))
        
        time_idx = self.get_time_index(time)
        trajectory = self.trajectory_cache['payload']
        
        position = trajectory['position'][time_idx, :]
        velocity = trajectory['velocity'][time_idx, :]
        acceleration = trajectory['acceleration'][time_idx, :]
        
        return (position, velocity, acceleration)
    
    def get_performance_stats(self) -> Dict:
        """获取性能统计"""
        if self.lookup_count == 0:
            return {'lookup_count': 0, 'avg_lookup_time': 0.0}
        
        avg_lookup_time = self.total_lookup_time / self.lookup_count * 1000  # ms
        return {
            'lookup_count': self.lookup_count,
            'total_lookup_time': self.total_lookup_time,
            'avg_lookup_time': avg_lookup_time
        }
    
    def reset_performance_stats(self):
        """重置性能统计"""
        self.lookup_count = 0
        self.total_lookup_time = 0.0
    
    def get_reference_for_mpc(self, time_traj: float, angle_t: float, nq: int = 6):
        """为MPC获取参考轨迹（兼容原始接口）"""
        start_time = time.time()
        if self.config_angle_t is not None and not self._angle_warned:
            try:
                if abs(float(angle_t) - float(self.config_angle_t)) > 1e-6:
                    print(
                        f"⚠️ 预计算轨迹的 angle_t={self.config_angle_t} 与当前 angle_t={angle_t} 不一致"
                    )
                    self._angle_warned = True
            except Exception:
                pass
        
        # 参数设置
        nxl = 13  # 载荷状态维度
        nul = nq   # 载荷控制维度
        nxi = 13  # 无人机状态维度
        nui = 4   # 无人机控制维度
        horizon = self.horizon
        
        # 初始化输出
        Ref_xq = []  # 无人机状态参考轨迹
        Ref_uq = []  # 无人机控制参考轨迹
        Ref_xl = np.zeros((nxl, horizon + 1))  # 载荷状态参考轨迹
        Ref_ul = np.zeros((nul, horizon))      # 载荷控制参考轨迹
        Ref0_xq = []  # 当前无人机参考状态
        Ref0_l = np.zeros((nxl, 1))  # 当前载荷参考状态
        
        # 获取时间索引
        time_idx = self.get_time_index(time_traj)
        
        # 为每架无人机计算参考轨迹
        for i in range(nq):
            Ref_xi = np.zeros((nxi, horizon + 1))
            Ref_ui = np.zeros((nui, horizon))
            
            # 获取无人机轨迹数据
            quad_key = f"quad_{i}"
            if quad_key in self.trajectory_cache:
                quad_trajectory = self.trajectory_cache[quad_key]
                
                for j in range(horizon + 1):
                    current_time_idx = min(time_idx + j, len(self.time_vector) - 1)
                    
                    # 无人机状态和控制
                    Ref_xi[:, j:j+1] = quad_trajectory['reference_state'][current_time_idx, :nxi].reshape(-1, 1)
                    if j < horizon:
                        Ref_ui[:, j:j+1] = quad_trajectory['reference_control'][current_time_idx, :nui].reshape(-1, 1)
            
            Ref_xq.append(Ref_xi)
            Ref_uq.append(Ref_ui)
            Ref0_xq.append(Ref_xi[:, 0:1])
        
        # 获取载荷轨迹数据
        if 'payload' in self.trajectory_cache:
            payload_trajectory = self.trajectory_cache['payload']
            
            for j in range(horizon + 1):
                current_time_idx = min(time_idx + j, len(self.time_vector) - 1)
                
                # 载荷状态和控制
                Ref_xl[:, j:j+1] = payload_trajectory['reference_state'][current_time_idx, :nxl].reshape(-1, 1)
                if j < horizon:
                    Ref_ul[:, j:j+1] = payload_trajectory['reference_control'][current_time_idx, :nul].reshape(-1, 1)
        
        # 当前载荷参考状态
        Ref0_l = Ref_xl[:, 0:1]
        
        # 更新性能统计
        end_time = time.time()
        self.lookup_count += 1
        self.total_lookup_time += (end_time - start_time)
        
        return Ref_xq, Ref_uq, Ref_xl, Ref_ul, Ref0_xq, Ref0_l
    
    def _find_closest_time(self, target_time: float) -> float:
        """找到最接近目标时间的可用时间点"""
        if self.time_vector is None:
            return target_time
        
        closest_idx = np.argmin(np.abs(self.time_vector - target_time))
        return self.time_vector[closest_idx]
    
    def print_performance_stats(self):
        """打印性能统计信息"""
        if self.lookup_count > 0:
            avg_lookup_time_ms = (self.total_lookup_time / self.lookup_count) * 1000
            print(f"--- 预计算轨迹管理器性能统计 ---")
            print(f"查找次数: {self.lookup_count}")
            print(f"总查找时间: {self.total_lookup_time:.6f} 秒")
            print(f"平均查找时间: {avg_lookup_time_ms:.6f} 毫秒")
        else:
            print("未执行轨迹查找。")


# 兼容性包装器 - 用于替换现有的Reference_for_MPC函数
class ReferenceTrajectoryWrapper:
    """参考轨迹包装器 - 提供与现有系统兼容的接口"""
    
    def __init__(self, precomputed_manager: PrecomputedTrajectoryManager):
        self.manager = precomputed_manager
    
    def Reference_for_MPC(self, time_traj: float, angle_t: float) -> Tuple:
        """
        兼容现有的Reference_for_MPC接口
        
        Returns:
            (Ref_xq, Ref_uq, Ref_xl, Ref_ul, Ref0_xq, Ref0_l)
        """
        return self.manager.get_reference_for_mpc(time_traj, angle_t)


def test_precomputed_trajectory_manager():
    """测试预计算轨迹管理器"""
    print("=== 测试预计算轨迹管理器 ===\n")
    
    # 初始化管理器
    manager = PrecomputedTrajectoryManager(
        data_path="precomputed_trajectories",
        horizon=10,
        dt_ctrl=0.02
    )
    
    # 测试单点查找
    print("1. 测试单点查找")
    test_times = [0.0, 5.0, 10.0, 15.0, 19.0]
    
    for t in test_times:
        for q in range(6):
            pos, vel, acc = manager.get_quadrotor_trajectory_point(t, q)
            print(f"时间 {t}s, 无人机 {q}: 位置={pos[:3]}")
    
    # 测试MPC参考轨迹
    print("\n2. 测试MPC参考轨迹")
    wrapper = ReferenceTrajectoryWrapper(manager)
    
    for t in test_times:
        try:
            Ref_xq, Ref_uq, Ref_xl, Ref_ul, Ref0_xq, Ref0_l = wrapper.Reference_for_MPC(t, 0.1)
            print(f"时间 {t}s: 生成 {len(Ref_xq)} 架无人机的参考轨迹")
        except ValueError as e:
            print(f"时间 {t}s: {e}")
    
    # 性能统计
    print("\n3. 性能统计")
    stats = manager.get_performance_stats()
    print(f"查找次数: {stats['lookup_count']}")
    print(f"平均查找时间: {stats['avg_lookup_time']:.3f} ms")


if __name__ == "__main__":
    test_precomputed_trajectory_manager()

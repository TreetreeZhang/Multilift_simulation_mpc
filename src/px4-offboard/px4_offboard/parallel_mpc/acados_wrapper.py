import os
import time
from typing import Tuple, List

import numpy as np

from ..config.parameter_manager import ParameterManager
from ..dynamics import multilifting
from ..control import MPC


class AcadosMPCSolver:
    """
    acados 求解器封装
    - 每个 worker 拥有独立的 AcadosOcpSolver 实例
    - Use the shared control.MPC ROS2 acados entry point
    """

    def __init__(self, agent_index: int, config_path: str | None = None, num_agents: int | None = None):
        self.agent_index = int(agent_index)
        self.params = ParameterManager(config_path).params
        if num_agents is not None:
            self.params.uav.num_uavs = int(num_agents)
        self.dt_ctrl = float(self.params.control.dt_ctrl)
        self.horizon = int(self.params.control.horizon)

        os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
        os.environ.setdefault("OMP_NUM_THREADS", "1")

        uav_para = self.params.uav.to_array()
        load_para = self.params.payload.to_mass_array()
        cable_para = self.params.cable.to_array()

        self.dyn = multilifting(uav_para, load_para, cable_para, self.dt_ctrl)
        self.dyn.model()

        self.mpc = MPC(
            uav_para=uav_para,
            load_para=load_para,
            cable_para=cable_para,
            dt_ctrl=self.dt_ctrl,
            horizon=self.horizon,
            gamma=self.params.parallel_mpc.gamma,
            gamma2=self.params.parallel_mpc.gamma2,
        )

        self._init_mpc_symbols()
        self._init_acados_solver()

        self.nxi = self.mpc.n_xi
        self.nui = self.mpc.n_ui
        self.nxl = self.mpc.n_xl
        self.nul = self.mpc.n_ul

        self.para_i = np.array(self.params.parallel_mpc.quad_weightings, dtype=np.float64).flatten()
        if self.para_i.size != self.mpc.n_pi:
            raise ValueError(
                f"quad_weightings length mismatch: expected {self.mpc.n_pi}, got {self.para_i.size}"
            )

    def _init_mpc_symbols(self) -> None:
        self.mpc.SetStateVariable(self.dyn.xi, self.dyn.xq, self.dyn.xl, self.dyn.index_q)
        self.mpc.SetCtrlVariable(self.dyn.ui, self.dyn.ul, self.dyn.ti)
        self.mpc.SetLoadParameter(self.dyn.Jldiag, self.dyn.rg)
        self.mpc.SetDyn(self.dyn.dyni, self.dyn.dynl, self.dyn.dyni, self.dyn.dynl)
        self.mpc.SetLearnablePara()
        self.mpc.SetQuadrotorCostDyn()
        self.mpc.SetConstraints_Qaudrotor()

    def _init_acados_solver(self) -> None:
        if "ACADOS_SOURCE_DIR" not in os.environ:
            raise EnvironmentError("ACADOS_SOURCE_DIR 未设置，无法初始化 acados 求解器")
        self.mpc.MPCsolverQuadrotorInit_ros2_acados(self.agent_index)

    def solve(self,
              xi_fb: np.ndarray,
              iter_xq: np.ndarray,
              ref_xq: np.ndarray,
              ref_uq: np.ndarray,
              iter_xl: np.ndarray,
              iter_ul: np.ndarray) -> Tuple[np.ndarray, np.ndarray, int, int]:
        """
        输入:
          - xi_fb:  [nxi]
          - iter_xq: [nq, N+1, nxi]
          - ref_xq:  [nq, N+1, nxi]
          - ref_uq:  [nq, N, nui]
          - iter_xl: [N+1, nxl]
          - iter_ul: [N, nul]
        输出:
          - xi_opt:          [N+1, nxi]
          - ui_opt:          [N, nui]
          - status:          0=OK, 非0=错误/超时
          - compute_time_us: 计算耗时（微秒）
        """
        t0 = time.perf_counter_ns()

        xi_fb = np.asarray(xi_fb, dtype=np.float64).reshape(-1)
        Ref_xi = np.asarray(ref_xq[self.agent_index], dtype=np.float64).reshape(-1)
        Ref_ui = np.asarray(ref_uq[self.agent_index], dtype=np.float64).reshape(-1)
        xl_traj = np.asarray(iter_xl, dtype=np.float64).reshape(-1)

        num_agents = int(iter_xq.shape[0])
        if iter_ul.shape[1] < num_agents:
            raise ValueError("iter_ul 列数不足以覆盖所有无人机")
        ul_traj = np.asarray(iter_ul[:, self.agent_index], dtype=np.float64).reshape(-1)

        other_indices = [i for i in range(num_agents) if i != self.agent_index]
        xqi_segments: List[np.ndarray] = []
        for j in other_indices:
            xy = np.asarray(iter_xq[j][:, 0:2], dtype=np.float64)
            xqi_segments.append(xy.reshape(-1))
        xqi_traj = np.concatenate(xqi_segments) if xqi_segments else np.zeros((0,), dtype=np.float64)

        result = self.mpc.MPCsolverQuadrotor_ros2_acados(
            xi_fb=xi_fb,
            xqi_traj=xqi_traj,
            xl_traj=xl_traj,
            ul_traj=ul_traj,
            Ref_xi=Ref_xi,
            Ref_ui=Ref_ui,
            Para_i=self.para_i,
            index=self.agent_index,
        )

        xi_opt = np.asarray(result["xi_opt"], dtype=np.float32)
        ui_opt = np.asarray(result["ui_opt"], dtype=np.float32)
        status = int(result.get("status", 0))
        t1 = time.perf_counter_ns()
        compute_time_us = int((t1 - t0) // 1000)
        return xi_opt, ui_opt, status, compute_time_us

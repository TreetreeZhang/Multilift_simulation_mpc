from typing import Tuple

import numpy as np

from .shared_memory import SharedArrays, SharedBarrier
from .worker import WorkerProcess
from ..config.parameter_manager import ParameterManager
from .profiling import CycleProfiler


class ParallelMPCCoordinator:
    """
    并行 MPC 协调器
    - 负责：共享内存与 worker 启动、周期同步、参考生成、结果汇总
    """

    def __init__(self, config_path: str | None = None, num_agents: int | None = None):
        params = ParameterManager(config_path).params
        self.num_agents = int(num_agents if num_agents is not None else params.uav.num_uavs)
        self.deadline_ms = int(params.parallel_mpc.deadline_ms)
        self.horizon = int(params.control.horizon)

        self.nxi = 13
        self.nui = 4
        self.nxl = 13
        self.nul = self.num_agents

        self.shm = SharedArrays(
            self.num_agents,
            self.nxi,
            self.nui,
            self.nxl,
            self.nul,
            self.horizon,
        )
        self.barrier = SharedBarrier(self.num_agents)
        self.profiler = CycleProfiler()

        self.workers = [
            WorkerProcess(i, self.num_agents, self.shm, self.barrier, self.nxi, self.nui, self.nxl, self.nul, self.horizon)
            for i in range(self.num_agents)
        ]
        for w in self.workers:
            w.start()

    def shutdown(self) -> None:
        for w in self.workers:
            if w.is_alive():
                w.terminate()
        self.shm.close()
        self.barrier.reset()

    def _fill_inputs(
        self,
        states: np.ndarray,
        iter_xq: np.ndarray,
        ref_xq: np.ndarray,
        ref_uq: np.ndarray,
        iter_xl: np.ndarray,
        iter_ul: np.ndarray,
    ) -> None:
        """
        写入本周期的状态、当前迭代轨迹与参考轨迹。
        """
        assert states.shape == (self.num_agents, self.nxi)
        assert iter_xq.shape == (self.num_agents, self.horizon + 1, self.nxi)
        assert ref_xq.shape == (self.num_agents, self.horizon + 1, self.nxi)
        assert ref_uq.shape == (self.num_agents, self.horizon, self.nui)
        assert iter_xl.shape == (self.horizon + 1, self.nxl)
        assert iter_ul.shape == (self.horizon, self.nul)

        np.copyto(self.shm.state_buffer.back(), states)
        np.copyto(self.shm.iter_xq.back(), iter_xq)
        np.copyto(self.shm.ref_xq.back(), ref_xq)
        np.copyto(self.shm.ref_uq.back(), ref_uq)
        np.copyto(self.shm.iter_xl.back(), iter_xl)
        np.copyto(self.shm.iter_ul.back(), iter_ul)

        self.shm.state_buffer.flip()
        self.shm.iter_xq.flip()
        self.shm.ref_xq.flip()
        self.shm.ref_uq.flip()
        self.shm.iter_xl.flip()
        self.shm.iter_ul.flip()

    def _collect_outputs(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        xq = self.shm.output_xq.front().copy()
        uq = self.shm.output_uq.front().copy()
        status = self.shm.output_status.front().copy()
        t_us = self.shm.output_time_us.front().copy()
        self.shm.output_xq.flip()
        self.shm.output_uq.flip()
        self.shm.output_status.flip()
        self.shm.output_time_us.flip()
        return xq, uq, status, t_us

    def run_one_cycle(
        self,
        states: np.ndarray,
        iter_xq: np.ndarray,
        ref_xq: np.ndarray,
        ref_uq: np.ndarray,
        iter_xl: np.ndarray,
        iter_ul: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        执行一个控制周期：
        - 写入输入
        - 唤醒 workers
        - 等待屏障或超时
        - 收集输出
        """
        self.profiler.begin()
        self._fill_inputs(states, iter_xq, ref_xq, ref_uq, iter_xl, iter_ul)

        self.barrier.signal_new_cycle()
        ok = self.barrier.coordinator_wait_all_done(timeout=self.deadline_ms / 1000.0)
        if not ok:
            raise TimeoutError(f"Parallel MPC workers exceeded {self.deadline_ms} ms deadline")

        self.barrier.new_cycle.clear()
        return self._collect_outputs()

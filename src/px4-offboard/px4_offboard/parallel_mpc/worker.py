import os
import signal
from multiprocessing import Process

from .shared_memory import SharedArrays, SharedBarrier
from .acados_wrapper import AcadosMPCSolver


def _setup_env_for_worker() -> None:
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("OMP_NUM_THREADS", "1")


def mpc_worker_entrypoint(agent_index: int,
                          num_agents: int,
                          shm: SharedArrays,
                          barrier: SharedBarrier,
                          nxi: int,
                          nui: int,
                          nxl: int,
                          nul: int,
                          horizon: int) -> None:
    """
    Worker 进程入口：
    - 等待 new_cycle
    - 读取 front 缓冲中的 state/ref
    - 调用求解器，写回 output 缓冲
    - 置位 done
    """
    _setup_env_for_worker()
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    solver = AcadosMPCSolver(agent_index, num_agents=num_agents)

    while True:
        barrier.worker_wait_for_cycle()

        xi_fb = shm.state_buffer.front()[agent_index]
        iter_xq = shm.iter_xq.front()
        ref_xq = shm.ref_xq.front()
        ref_uq = shm.ref_uq.front()
        iter_xl = shm.iter_xl.front()
        iter_ul = shm.iter_ul.front()

        xi_opt, ui_opt, status, compute_time_us = solver.solve(
            xi_fb=xi_fb,
            iter_xq=iter_xq,
            ref_xq=ref_xq,
            ref_uq=ref_uq,
            iter_xl=iter_xl,
            iter_ul=iter_ul,
        )

        shm.output_xq.front()[agent_index] = xi_opt
        shm.output_uq.front()[agent_index] = ui_opt
        shm.output_status.front()[agent_index] = status
        shm.output_time_us.front()[agent_index] = compute_time_us

        barrier.worker_signal_done()


class WorkerProcess(Process):
    def __init__(self,
                 agent_index: int,
                 num_agents: int,
                 shm: SharedArrays,
                 barrier: SharedBarrier,
                 nxi: int,
                 nui: int,
                 nxl: int,
                 nul: int,
                 horizon: int):
        super().__init__(
            target=mpc_worker_entrypoint,
            args=(agent_index, num_agents, shm, barrier, nxi, nui, nxl, nul, horizon),
            daemon=True,
        )
        self.agent_index = agent_index

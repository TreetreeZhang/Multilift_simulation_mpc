import ctypes
from multiprocessing import shared_memory, Event, Value
from typing import Tuple, Optional

import numpy as np


class SharedBarrier:
    """
    简单的双事件屏障：
    - coordinator 设置 new_cycle 事件，workers 等待
    - workers 完成后递增计数，并在 all_done 事件上唤醒 coordinator
    """

    def __init__(self, num_workers: int):
        self.num_workers = num_workers
        self.new_cycle = Event()
        self.all_done = Event()
        self.done_count = Value(ctypes.c_int, 0, lock=True)

    def signal_new_cycle(self) -> None:
        with self.done_count.get_lock():
            self.done_count.value = 0
        self.all_done.clear()
        self.new_cycle.set()

    def worker_wait_for_cycle(self) -> None:
        self.new_cycle.wait()

    def worker_signal_done(self) -> None:
        with self.done_count.get_lock():
            self.done_count.value += 1
            if self.done_count.value >= self.num_workers:
                self.all_done.set()

    def coordinator_wait_all_done(self, timeout: Optional[float]) -> bool:
        return self.all_done.wait(timeout=timeout)

    def reset(self) -> None:
        self.new_cycle.clear()
        self.all_done.clear()
        with self.done_count.get_lock():
            self.done_count.value = 0


class SharedDoubleBuffer:
    """
    双缓冲共享数组：
    - 两个物理缓冲轮换（front/back），通过 seq_id 区分
    - 提供 numpy 视图进行零拷贝读写
    """

    def __init__(self, shape: Tuple[int, ...], dtype: np.dtype):
        self.shape = tuple(shape)
        self.dtype = np.dtype(dtype)
        self.nbytes = int(np.prod(self.shape)) * self.dtype.itemsize

        self._shm0 = shared_memory.SharedMemory(create=True, size=self.nbytes)
        self._shm1 = shared_memory.SharedMemory(create=True, size=self.nbytes)
        self._buf0 = np.ndarray(self.shape, dtype=self.dtype, buffer=self._shm0.buf)
        self._buf1 = np.ndarray(self.shape, dtype=self.dtype, buffer=self._shm1.buf)

        self._front_index = Value(ctypes.c_int, 0, lock=True)
        self.seq_id = Value(ctypes.c_ulonglong, 0, lock=True)

    def front(self) -> np.ndarray:
        return self._buf0 if self._front_index.value == 0 else self._buf1

    def back(self) -> np.ndarray:
        return self._buf1 if self._front_index.value == 0 else self._buf0

    def flip(self) -> int:
        with self._front_index.get_lock(), self.seq_id.get_lock():
            self._front_index.value = 1 - self._front_index.value
            self.seq_id.value += 1
            return self._front_index.value

    def close(self) -> None:
        self._shm0.close()
        self._shm1.close()
        try:
            self._shm0.unlink()
        except FileNotFoundError:
            pass
        try:
            self._shm1.unlink()
        except FileNotFoundError:
            pass

    @property
    def name_pair(self) -> Tuple[str, str]:
        return (self._shm0.name, self._shm1.name)


class SharedArrays:
    """
    并行 MPC 共享数组集合：
    - state_buffer:   [N, nxi]
    - iter_xq:        [N, horizon+1, nxi]
    - ref_xq:         [N, horizon+1, nxi]
    - ref_uq:         [N, horizon, nui]
    - iter_xl:        [horizon+1, nxl]
    - iter_ul:        [horizon, nul]
    - output_xq:      [N, horizon+1, nxi]
    - output_uq:      [N, horizon, nui]
    - output_status:  [N] int32
    - output_time_us: [N] int32
    """

    def __init__(self,
                 num_agents: int,
                 nxi: int,
                 nui: int,
                 nxl: int,
                 nul: int,
                 horizon: int):
        self.num_agents = num_agents
        self.nxi = nxi
        self.nui = nui
        self.nxl = nxl
        self.nul = nul
        self.horizon = horizon

        self.state_buffer = SharedDoubleBuffer((num_agents, nxi), np.float32)
        self.iter_xq = SharedDoubleBuffer((num_agents, horizon + 1, nxi), np.float32)
        self.ref_xq = SharedDoubleBuffer((num_agents, horizon + 1, nxi), np.float32)
        self.ref_uq = SharedDoubleBuffer((num_agents, horizon, nui), np.float32)
        self.iter_xl = SharedDoubleBuffer((horizon + 1, nxl), np.float32)
        self.iter_ul = SharedDoubleBuffer((horizon, nul), np.float32)
        self.output_xq = SharedDoubleBuffer((num_agents, horizon + 1, nxi), np.float32)
        self.output_uq = SharedDoubleBuffer((num_agents, horizon, nui), np.float32)
        self.output_status = SharedDoubleBuffer((num_agents,), np.int32)
        self.output_time_us = SharedDoubleBuffer((num_agents,), np.int32)

    def close(self) -> None:
        self.state_buffer.close()
        self.iter_xq.close()
        self.ref_xq.close()
        self.ref_uq.close()
        self.iter_xl.close()
        self.iter_ul.close()
        self.output_xq.close()
        self.output_uq.close()
        self.output_status.close()
        self.output_time_us.close()

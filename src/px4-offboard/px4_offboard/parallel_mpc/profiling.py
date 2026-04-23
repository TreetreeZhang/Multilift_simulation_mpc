import time
from dataclasses import dataclass


@dataclass
class CycleTiming:
    t_start_ns: int
    t_distribute_ns: int
    t_solve_ns: int
    t_publish_ns: int


class CycleProfiler:
    def __init__(self):
        self._last = None

    @staticmethod
    def now_ns() -> int:
        return time.monotonic_ns()

    def begin(self) -> int:
        self._last = self.now_ns()
        return self._last

    def elapsed_us(self, since_ns: int) -> int:
        return int((self.now_ns() - since_ns) // 1000)

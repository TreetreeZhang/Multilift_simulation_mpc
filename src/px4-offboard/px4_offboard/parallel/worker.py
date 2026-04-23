"""Parallel MPC worker module."""

try:
    from px4_offboard.parallel_mpc.worker import WorkerProcess, mpc_worker_entrypoint
except ImportError:
    from parallel_mpc.worker import WorkerProcess, mpc_worker_entrypoint

__all__ = ["WorkerProcess", "mpc_worker_entrypoint"]

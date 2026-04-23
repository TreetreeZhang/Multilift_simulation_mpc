try:
    from px4_offboard.parallel.coordinator import ParallelMPCCoordinator
    from px4_offboard.parallel.worker import WorkerProcess, mpc_worker_entrypoint
except ImportError:
    from .coordinator import ParallelMPCCoordinator
    from .worker import WorkerProcess, mpc_worker_entrypoint

__all__ = ["ParallelMPCCoordinator", "WorkerProcess", "mpc_worker_entrypoint"]

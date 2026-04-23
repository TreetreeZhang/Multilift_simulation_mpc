"""Parallel MPC coordinator module."""

try:
    from px4_offboard.parallel_mpc.coordinator import ParallelMPCCoordinator
except ImportError:
    from parallel_mpc.coordinator import ParallelMPCCoordinator

__all__ = ["ParallelMPCCoordinator"]

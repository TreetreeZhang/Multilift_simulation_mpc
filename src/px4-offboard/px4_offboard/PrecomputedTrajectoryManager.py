"""Backward-compatible trajectory manager module.

New code should import from :mod:`px4_offboard.trajectory.trajectory_manager`.
"""

try:
    from px4_offboard.trajectory.trajectory_manager import (
        PrecomputedTrajectoryManager,
        ReferenceTrajectoryWrapper,
        test_precomputed_trajectory_manager,
    )
except ImportError:
    from trajectory.trajectory_manager import (
        PrecomputedTrajectoryManager,
        ReferenceTrajectoryWrapper,
        test_precomputed_trajectory_manager,
    )

__all__ = [
    "PrecomputedTrajectoryManager",
    "ReferenceTrajectoryWrapper",
    "test_precomputed_trajectory_manager",
]

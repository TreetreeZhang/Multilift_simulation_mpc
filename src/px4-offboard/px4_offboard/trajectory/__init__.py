try:
    from px4_offboard.trajectory.trajectory_manager import (
        PrecomputedTrajectoryManager,
        ReferenceTrajectoryWrapper,
    )
except ImportError:
    from .trajectory_manager import PrecomputedTrajectoryManager, ReferenceTrajectoryWrapper

__all__ = ["PrecomputedTrajectoryManager", "ReferenceTrajectoryWrapper"]

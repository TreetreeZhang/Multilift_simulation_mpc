"""Backward-compatible reference trajectory generator module."""

try:
    from px4_offboard.trajectory.reference_trajectory import *  # noqa: F401,F403
except ImportError:
    from trajectory.reference_trajectory import *  # noqa: F401,F403

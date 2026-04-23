"""Backward-compatible dynamics module.

New code should import from :mod:`px4_offboard.dynamics`.
"""

try:
    from px4_offboard.dynamics.quadrotor import multilifting
except ImportError:
    from dynamics.quadrotor import multilifting

__all__ = ["multilifting"]

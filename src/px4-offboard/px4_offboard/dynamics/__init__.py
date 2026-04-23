try:
    from px4_offboard.dynamics.quadrotor import multilifting
except ImportError:
    from .quadrotor import multilifting

__all__ = ["multilifting"]

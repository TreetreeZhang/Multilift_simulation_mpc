"""Payload dynamics helpers.

Payload-related methods currently live on the shared ``multilifting`` model.
"""

try:
    from px4_offboard.dynamics.quadrotor import multilifting
except ImportError:
    from .quadrotor import multilifting

__all__ = ["multilifting"]

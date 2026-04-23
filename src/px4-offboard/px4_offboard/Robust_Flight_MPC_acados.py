"""Backward-compatible control module.

New code should import from :mod:`px4_offboard.control`.
"""

try:
    from px4_offboard.control import (
        Controller,
        MPC,
        MPC_gradient,
        Sensitivity_propagation,
    )
except ImportError:
    from control import Controller, MPC, MPC_gradient, Sensitivity_propagation

__all__ = ["Controller", "MPC", "MPC_gradient", "Sensitivity_propagation"]

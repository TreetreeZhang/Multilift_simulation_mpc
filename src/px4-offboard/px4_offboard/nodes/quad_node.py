"""Thin ROS2 quadrotor node entry point."""

try:
    from px4_offboard.multilift_quad_node import QuadNode, main
except ImportError:
    from multilift_quad_node import QuadNode, main

__all__ = ["QuadNode", "main"]

"""Thin ROS2 central coordinator node entry point."""

try:
    from px4_offboard.multilift_sync_node import SyncNode, main
except ImportError:
    from multilift_sync_node import SyncNode, main

CentralNode = SyncNode
__all__ = ["CentralNode", "SyncNode", "main"]

#!/usr/bin/env python3
"""payload_sitl_ros.py – drones init + payload pose pub (50 Hz) – fixed pub list"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

import omni.timeline as tl
from omni.isaac.core import World
import omni.isaac.core.utils.prims as prim_utils
from pxr import UsdGeom, UsdPhysics, Gf, PhysxSchema
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend, PX4MavlinkBackendConfig)
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from cable_model import RigidBodyRopes
from scipy.spatial.transform import Rotation
from pathlib import Path
import rclpy
from geometry_msgs.msg import TransformStamped, PoseStamped, TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from time import sleep
import yaml


def load_sim_params():
    current_dir = Path(__file__).resolve().parent
    candidates = [
        current_dir.parent.parent / "px4-offboard" / "px4_offboard" / "config" / "multilift_params.yaml",
        current_dir.parent.parent.parent / "install" / "px4_offboard" / "share" / "px4_offboard" / "config" / "multilift_params.yaml",
    ]

    for path in candidates:
        if path.exists():
            with path.open("r", encoding="utf-8") as f:
                config = yaml.safe_load(f) or {}
            return {
                "num_drones": int(config.get("uav", {}).get("num_uavs", 6)),
                "payload_mass": float(config.get("payload", {}).get("mass", 6.0)),
                "rope_length": float(config.get("cable", {}).get("rest_length", 1.8)),
            }

    return {
        "num_drones": 6,
        "payload_mass": 6.0,
        "rope_length": 1.8,
    }


SIM_PARAMS = load_sim_params()
NUM_DRONES = SIM_PARAMS["num_drones"]
PAYLOAD_MASS = SIM_PARAMS["payload_mass"]
ROPE_LENGTH = SIM_PARAMS["rope_length"]
PUB_HZ = 50.0
LOAD_HEIGHT = 0.10
DRONE_SPAWN_Z_OFFSET = 0.10

# ───────────────────────────────── world ────────────────────────────
class Sim:
    def __init__(self):
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        phys = self.world.get_physics_context(); phys.enable_gpu_dynamics(True)
        PhysxSchema.PhysxSceneAPI.Apply(self.world.stage.GetPrimAtPath("/physicsScene"))
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Flat Plane"])
        prim_utils.create_prim("/World/Light_Key", "SphereLight",
                               position=Gf.Vec3f(0, 0, 55),
                               attributes={"inputs:radius": 30.0,
                                           "inputs:intensity": 1e4})

# ─────────────────────────────── scene ─────────────────────────────
def spawn(sim, node, init_pubs):
    stg = sim.world.stage
    RigidBodyRopes().create(stg, num_ropes=NUM_DRONES,
                            rope_length=ROPE_LENGTH, payload_mass=PAYLOAD_MASS, load_height=LOAD_HEIGHT)
    payload = stg.GetPrimAtPath("/World/CommonPayload/Payload")
    xf = UsdGeom.XformCache(); sleep(1)               # wait for USD

    for i in range(NUM_DRONES):
        box  = f"/World/Rope{i}/box{i}Actor"
        pos  = xf.GetLocalToWorldTransform(stg.GetPrimAtPath(box)).ExtractTranslation()
        dpos = pos + Gf.Vec3d(0, 0, DRONE_SPAWN_Z_OFFSET)

        cfg = MultirotorConfig()
        cfg.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
            "vehicle_id": i,
            "px4_autolaunch": True,
            "px4_dir": sim.pg.px4_path,
            "px4_vehicle_model": sim.pg.px4_default_airframe,
        }))]

        prim = "/World/quadrotor" if i == 0 else f"/World/quadrotor_{i:02d}"
        Multirotor(prim, ROBOTS["Iris"], i, dpos, Rotation.identity().as_quat(),
                   config=cfg)

        joint = UsdPhysics.FixedJoint.Define(stg, f"/World/Rope{i}/droneJoint")
        joint.CreateBody0Rel().SetTargets([box])
        joint.CreateBody1Rel().SetTargets([f"{prim}/body"])
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -0.06))

        # publish init pose immediately
        msg = TransformStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "world"; msg.child_frame_id = f"drone_{i}"
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, dpos)
        msg.transform.rotation.w = 1.0
        init_pubs[i].publish(msg)

    # ensure ROS2 sends
    rclpy.spin_once(node, timeout_sec=0.0)
    return payload

# ─────────────────────────────── main ──────────────────────────────
def main():
    rclpy.init(); node = rclpy.create_node("sitl_pose_pub")

    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )

    init_pubs = [node.create_publisher(TransformStamped,
                                       f"/drone_{i}_init_pos", 1)
                 for i in range(NUM_DRONES)]
    pay_pose_pub = node.create_publisher(PoseStamped, "/payload_pose", qos)
    pay_twist_pub = node.create_publisher(TwistStamped, "/payload_twist", qos)

    sim     = Sim()
    payload = spawn(sim, node, init_pubs)

    dt          = sim.world.get_physics_dt()
    step_target = max(1, round(1.0 / (PUB_HZ * dt)))
    steps       = 0
    xf_cache    = UsdGeom.XformCache()          # reuse, Clear() each pub
    publish_dt  = step_target * dt
    pay_pose_msg = PoseStamped(); pay_pose_msg.header.frame_id = "world"
    pay_twist_msg = TwistStamped(); pay_twist_msg.header.frame_id = "world"
    prev_pos = None

    def cb(_):                                  # physics callback
        nonlocal steps, prev_pos
        steps += 1
        if steps < step_target:
            return
        steps = 0

        xf_cache.Clear()
        tf   = xf_cache.GetLocalToWorldTransform(payload)
        pos  = tf.ExtractTranslation()
        quat = tf.ExtractRotation().GetQuat()

        curr_pos = [float(pos[0]), float(pos[1]), float(pos[2])]
        if prev_pos is None:
            lin_vel = [0.0, 0.0, 0.0]
        else:
            lin_vel = [
                (curr_pos[0] - prev_pos[0]) / publish_dt,
                (curr_pos[1] - prev_pos[1]) / publish_dt,
                (curr_pos[2] - prev_pos[2]) / publish_dt,
            ]
        prev_pos = curr_pos

        stamp = node.get_clock().now().to_msg()
        pay_pose_msg.header.stamp = stamp
        pay_pose_msg.pose.position.x = curr_pos[0]
        pay_pose_msg.pose.position.y = curr_pos[1]
        pay_pose_msg.pose.position.z = curr_pos[2]
        imag = quat.GetImaginary()
        pay_pose_msg.pose.orientation.x = float(imag[0])
        pay_pose_msg.pose.orientation.y = float(imag[1])
        pay_pose_msg.pose.orientation.z = float(imag[2])
        pay_pose_msg.pose.orientation.w = float(quat.GetReal())

        pay_twist_msg.header.stamp = stamp
        pay_twist_msg.twist.linear.x = lin_vel[0]
        pay_twist_msg.twist.linear.y = lin_vel[1]
        pay_twist_msg.twist.linear.z = lin_vel[2]
        pay_twist_msg.twist.angular.x = 0.0
        pay_twist_msg.twist.angular.y = 0.0
        pay_twist_msg.twist.angular.z = 0.0

        pay_pose_pub.publish(pay_pose_msg)
        pay_twist_pub.publish(pay_twist_msg)

        rclpy.spin_once(node, timeout_sec=0.0)

    sim.world.add_physics_callback("payload_pub", cb)

    w = sim.world; tli = tl.get_timeline_interface()
    w.reset(); tli.play()
    while simulation_app.is_running(): w.step(render=False)
    tli.stop(); simulation_app.close()

    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()

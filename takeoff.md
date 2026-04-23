Step1:(Terminal 0)

conda deactivate

MicroXRCEAgent udp4 -p 8888

Step2:(new Terminal 1)
conda deactivate

pkill -f px4

export ROS_DISTRO=humble && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/carlson/.conda/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib 

source /home/carlson/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash

ISAACSIM_PYTHON src/sitl_sim/sitl_sim/sitl_stable.py

Step3:(new Terminal 2)
ROS2:

conda deactivate

source /opt/ros/humble/setup.bash

source install/local_setup.bash

ros2 launch px4_offboard multilift_mpc.launch.py

ros2 launch px4_offboard multilift_mpc.launch.py | grep "\[perf\]"


Tips:if you change your code, must "colcon build" in your ROS2 workspace
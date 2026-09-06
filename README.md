# 2dof_camera_control

2-DOF (yaw/pitch) Dynamixel camera mount for the Unitree G1 head.

- `python/` — standalone hardware tools: servo ID assignment, calibration, and CLI angle control. No ROS dependency.
- `ros2/` — ROS2 (Humble) workspace for controlling the mount over topics.

## ros2/ setup

`dynamixel-sdk` must be installed for **system** Python3 (not just a conda/venv), because
`ros2 run` executes built packages with a hardcoded `/usr/bin/python3` shebang regardless of
which environment ran `colcon build`:

```
pip3 install --user /path/to/DynamixelSDK/python
```

Then, from `ros2/`:

```
colcon build --packages-select camera_mount_bridge
source install/setup.bash
ros2 run camera_mount_bridge bridge_node --ros-args -p device:=/dev/ttyUSB0
```

Test with (no URDF/TF/RViz needed):
```
ros2 topic pub /joint_command sensor_msgs/msg/JointState "{name: ['yaw'], position: [0.3]}"
ros2 topic echo /joint_states
```

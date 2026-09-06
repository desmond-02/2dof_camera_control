# 2dof_camera_control

2-DOF (yaw/pitch) Dynamixel camera mount for the Unitree G1 head.

- `python/` — standalone hardware tools: servo ID assignment, calibration, and CLI angle control. No ROS dependency.
- `ros2/` — ROS2 (Humble) workspace for controlling the mount over topics.

## ros2/ setup

`dynamixel-sdk` must be installed for **system** Python3 (not conda/venv) — `colcon build`
for an `ament_cmake` package (like `camera_mount_description`) invokes whatever `python3` is
on `PATH` and needs `catkin_pkg`, while `ros2 run` executes built packages with a hardcoded
`/usr/bin/python3` shebang regardless of which environment ran the build. Simplest: don't
activate conda at all for this workspace, just:

```
pip3 install --user /path/to/DynamixelSDK/python
```

Then, from `ros2/`:

```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Packages:
- `camera_mount_bridge` — talks to the real servos. `ros2 run camera_mount_bridge bridge_node --ros-args -p device:=/dev/ttyUSB0`
  (also homes both joints to center on startup). Test with no URDF/TF/RViz needed:
  ```
  ros2 topic pub /joint_command sensor_msgs/msg/JointState "{name: ['yaw'], position: [0.3]}"
  ros2 topic echo /joint_states
  ```
- `camera_mount_description` — URDF/xacro (placeholder geometry until the OnShape design is
  exported; joint names/limits are real). `ros2 launch camera_mount_description display.launch.py`
  previews it with `joint_state_publisher_gui` sliders, no hardware involved.
- `camera_mount_bringup` — real hardware + TF together: `ros2 launch camera_mount_bringup
  bringup.launch.py device:=/dev/ttyUSB0` (add `rviz:=true` to also open RViz).

# 2dof_camera_control

2-DOF (yaw/pitch) Dynamixel camera mount for the Unitree G1 head.

- `python/` — standalone hardware tools: servo ID assignment, calibration, and CLI angle control. No ROS dependency.
- `ros2/` — ROS2 (Humble) workspace for controlling the mount over topics.

## Setting up on a new robot

Step-by-step for bringing this up on a machine that doesn't have it yet, assuming ROS2
Humble is already installed. Steps marked **(per-hardware)** need redoing if the physical
servos or U2D2 unit themselves ever change — everything else is one-time per machine.

1. **Clone this repo and DynamixelSDK** (a separate dependency, not vendored in here):
   ```bash
   git clone <this repo's URL> ~/CodeSpace/2dof_camera_control
   git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git ~/CodeSpace/DynamixelSDK
   ```
2. **Install dependencies.**
   `dynamixel-sdk` must go to **system** Python3, not conda/venv — `colcon build` for an
   `ament_cmake` package (like `camera_mount_description`) invokes whatever `python3` is on
   `PATH` and needs `catkin_pkg`, while `ros2 run` executes built packages with a hardcoded
   `/usr/bin/python3` shebang regardless of which environment ran the build. Simplest: don't
   activate conda at all for this workspace:
   ```bash
   pip3 install --user ~/CodeSpace/DynamixelSDK/python
   ```
   `camera_mount_description` also depends on `realsense2_description` (for the D455's real
   TF frames). Install it as a normal system package — `rosdep` will pick this up
   automatically from `package.xml`:
   ```bash
   sudo apt install ros-humble-realsense2-description
   ```
   (or, from `ros2/`: `rosdep install --from-paths src --ignore-src -r -y`, which resolves
   the same dependency and works for any other `exec_depend` this workspace picks up later
   too). No manual cloning or symlinking of `realsense-ros` should be necessary on a machine
   with normal apt/sudo access — that's only a fallback for sandboxes without it, and isn't
   part of this repo.
3. **Serial port permissions** (one-time per machine/user):
   ```bash
   groups $USER | grep dialout || sudo usermod -aG dialout $USER
   ```
   If you had to run the `usermod`, log out and back in — group membership doesn't apply
   to an already-open session.
4. **(per-hardware) Set up a stable device path for the U2D2.** Its `/dev/ttyUSBx` number
   isn't guaranteed to stay the same across reboots or replugs — it can silently shift from
   `/dev/ttyUSB0` to `/dev/ttyUSB1` depending on USB enumeration order. A udev rule fixes
   this by creating a permanent symlink (`/dev/dynamixel_pan_tilt`) keyed to the U2D2's own
   USB identity, so it always resolves to the right device regardless of which `ttyUSBx`
   number the kernel assigns it. Each physical U2D2 has its own USB serial number, so read
   it fresh rather than reusing another robot's rule:
   ```bash
   udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|ATTRS\{serial\}"
   ```
   Then install the rule with those values (needs `sudo`):
   ```bash
   sudo tee /etc/udev/rules.d/99-dynamixel-pan-tilt.rules > /dev/null <<'EOF'
   SUBSYSTEM=="tty", ATTRS{idVendor}=="<paste>", ATTRS{idProduct}=="<paste>", ATTRS{serial}=="<paste>", SYMLINK+="dynamixel_pan_tilt", MODE="0666", GROUP="dialout"
   EOF
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
   If `/dev/dynamixel_pan_tilt` doesn't appear right away, unplug and replug the U2D2 (some
   systems only apply a newly-added rule on the next USB enumeration event). Verify it
   worked:
   ```bash
   ls -la /dev/dynamixel_pan_tilt   # should show -> ttyUSBx (whichever number it actually is)
   ```
   The real test: unplug the U2D2, plug it into a **different** USB port, and confirm
   `/dev/dynamixel_pan_tilt` still exists and still points at whatever `ttyUSBx` it became.

   To roll back:
   ```bash
   sudo rm /etc/udev/rules.d/99-dynamixel-pan-tilt.rules
   sudo udevadm control --reload-rules
   ```
5. **Build the workspace**:
   ```bash
   cd ~/CodeSpace/2dof_camera_control/ros2
   source /opt/ros/humble/setup.bash
   colcon build
   source install/setup.bash
   ```
6. **(per-hardware) Calibrate — don't reuse `config/servos.json` from another robot.**
   `center_tick`/`min_limit`/`max_limit` encode this specific pair of servos' mechanical
   zero point and range of motion; they don't carry over to different servos, even
   identical model numbers, since it depends on exactly how each horn was mounted:
   ```bash
   cd ~/CodeSpace/2dof_camera_control/python
   python3 calibrate.py
   ```
   Copy the resulting values into `ros2/src/camera_mount_bridge/config/servos.json` (kept
   as its own separate copy from `python/config/servos.json` — see "Packages" below), then
   rebuild that package: `colcon build --packages-select camera_mount_bridge`.
7. **Verify hardware bring-up**:
   ```bash
   ros2 launch camera_mount_bringup bringup.launch.py rviz:=true
   ```
   Should work with no `device:=` override (thanks to step 4), home both joints to center,
   and show correct TF/RViz. Test a move:
   ```bash
   ros2 topic pub /joint_command sensor_msgs/msg/JointState "{name: ['yaw'], position: [0.3]}"
   ros2 topic echo /joint_states
   ```
8. **(later, separate task) Attach to this robot's own URDF.** Steps 1–7 get you a working
   mount reporting TF against a placeholder `world` root, same as any dev machine. Actually
   attaching it to a real robot (so its head motion carries the camera along) means
   measuring the offset from that robot's head/torso link to `camera_mount_base_link`, then
   adding a `xacro:camera_mount parent="...">` call into the robot's own URDF — see
   `camera_mount_description`'s own [README](ros2/src/camera_mount_description/README.md)
   for the full explanation. Not required for basic bring-up.

**Not covered by this repo**: if you want the D455's live image/depth stream (not just its
TF frames), you separately need `ros-humble-realsense2-camera` — `camera_mount_description`
only provides geometry and TF, not a running camera driver.

## Packages

- `camera_mount_bridge` — talks to the real servos. `ros2 run camera_mount_bridge bridge_node`
  (defaults to `/dev/dynamixel_pan_tilt` — see step 4 above; override with
  `--ros-args -p device:=/dev/ttyUSBx` if that symlink isn't installed) (also homes both
  joints to center on startup). Test with no URDF/TF/RViz needed:
  ```
  ros2 topic pub /joint_command sensor_msgs/msg/JointState "{name: ['yaw'], position: [0.3]}"
  ros2 topic echo /joint_states
  ```
- `camera_mount_description` — URDF/xacro exported from the real OnShape design, with a
  RealSense D455 attached via `realsense2_description`. See its own
  [README](ros2/src/camera_mount_description/README.md) for how to re-sync it against a fresh
  OnShape export, and how it attaches to a parent robot (the G1) later.
  `ros2 launch camera_mount_description display.launch.py` previews it with
  `joint_state_publisher_gui` sliders, no hardware involved.
- `camera_mount_bringup` — real hardware + TF together: `ros2 launch camera_mount_bringup
  bringup.launch.py` (defaults to `/dev/dynamixel_pan_tilt`, same as above; add
  `rviz:=true` to also open RViz, or `device:=/dev/ttyUSBx` to override).

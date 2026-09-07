# camera_mount_description

URDF/xacro for the 2-DOF (yaw/pitch) Dynamixel camera mount, exported from the real
OnShape design via [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot), with
a RealSense D455 attached via `realsense2_description`'s `sensor_d455` macro (real
`camera_link`/optical/IMU TF frames, not just decorative mesh).

## Layout

- `urdf/camera_mount.urdf.xacro` — the mount itself, packaged as a `xacro:macro` named
  `camera_mount` (params: `parent`, `*origin`). Include it and instantiate it from
  wherever it needs to attach:
  ```xml
  <xacro:include filename="$(find camera_mount_description)/urdf/camera_mount.urdf.xacro"/>
  <xacro:camera_mount parent="some_link">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:camera_mount>
  ```
- `urdf/camera_mount_standalone.urdf.xacro` — instantiates that macro under a free-floating
  `world` link, so `display.launch.py` and `camera_mount_bringup` have something to publish
  TF against before the mount is attached to a real robot.
- `urdf/assets/` — STL meshes referenced by the xacro.
- `onshape_export/onshape.urdf` + `config.json` — the raw, unedited onshape-to-robot export.
  Regenerated wholesale on every OnShape export; never hand-edited. Kept here purely as the
  reference to diff against when re-syncing (see below).

## Attaching to the G1 later

Once the mount's physical location on the G1 head is known, the G1's own URDF includes
`camera_mount.urdf.xacro` and instantiates `xacro:camera_mount` with `parent` set to the
appropriate G1 head link and `origin` set to the measured mounting offset — same pattern
as `camera_mount_standalone.urdf.xacro` above, just with a real parent instead of `world`.
`camera_mount_bringup`'s launch file would then load the G1's URDF instead of the standalone
wrapper.

## Re-syncing after a fresh OnShape export

Whenever the OnShape design changes and gets re-exported (`onshape.urdf` overwritten,
mesh STLs regenerated):

1. Copy the fresh `onshape.urdf` and any changed STLs over `onshape_export/onshape.urdf`
   and `urdf/assets/*.stl`.
2. Diff the fresh `onshape.urdf` against `urdf/camera_mount.urdf.xacro`, block by block:
   the three links (`camera_mount_base_link`, `camera_mount_yaw_link`,
   `camera_mount_pitch_link`) and the two joints (`yaw`, `pitch`). Copy every inertial/
   visual/collision `<origin>`, `<inertia>`, and `<mass>` value straight across — the
   structure never changes between exports, only the numbers.
3. **One deliberate difference to preserve**: inside `camera_mount_pitch_link`, the fresh
   export has a `"Part d455"` visual/collision block — do **not** copy that over. It stays
   dropped in the xacro; `xacro:sensor_d455` at the bottom of the file supplies the real
   mesh instead, correctly tied into TF (`camera_link` + friends) rather than floating
   disconnected from it.
4. **Check whether the D455 offset needs recomputing**: compare the `"Part d455"` visual's
   `<origin>` in the fresh export against what it was last time (noted in the xacro's own
   comment above `xacro:sensor_d455`). If those numbers changed, the camera moved relative
   to its holder in the CAD and the `xacro:sensor_d455` origin must be re-solved:
   ```
   T(pitch_link -> bottom_screw_frame) = T(pitch_link -> d455 mesh) . T(bottom_screw_frame -> d455 mesh)^-1
   ```
   using `realsense2_description`'s own `bottom_screw_frame -> mesh` offsets for the second
   term. If unchanged (the common case), leave `xacro:sensor_d455`'s origin as-is.
5. Rebuild and validate, in order:
   ```
   colcon build --packages-select camera_mount_description
   ```
   ```python
   import xacro
   doc = xacro.process_file('install/camera_mount_description/share/camera_mount_description/urdf/camera_mount_standalone.urdf.xacro')
   open('/tmp/check.urdf', 'w').write(doc.toxml())
   ```
   ```
   check_urdf /tmp/check.urdf
   ```
   confirms the xacro processes and the kinematic tree is intact before opening RViz.
6. Visually confirm: `ros2 launch camera_mount_description display.launch.py`.

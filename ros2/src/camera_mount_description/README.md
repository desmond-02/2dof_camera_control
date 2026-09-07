# camera_mount_description

URDF/xacro for the 2-DOF (yaw/pitch) Dynamixel camera mount, exported from the real
OnShape design via [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot), with
a RealSense D455 attached via `realsense2_description`'s `sensor_d455` macro (real
`camera_link`/optical/IMU TF frames, not just a decorative mesh).

## The TF tree

```
<parent link you attach to>
  │
  ├─[camera_mount_joint, FIXED]        origin = wherever you mount it
  ▼
camera_mount_base_link                 (bracket/housing)
  │
  ├─[yaw, REVOLUTE, axis z]
  ▼
camera_mount_yaw_link                  (yaw servo horn)
  │
  ├─[pitch, REVOLUTE, axis z]
  ▼
camera_mount_pitch_link                (pitch bracket, holds the camera)
  │
  ├─[xacro:sensor_d455 macro]          real RealSense TF frames, not a decoration
  ▼
camera_bottom_screw_frame → camera_link → {color, depth, infra1, infra2,
  gyro→imu, accel} frames + their _optical_frame counterparts
```

Everything from `camera_mount_base_link` down is fixed, hand-maintained content that
never changes based on what this mount is attached to. Only the very top — the
`camera_mount_joint` — depends on that.

## Layout

- **`urdf/camera_mount.urdf.xacro`** — the mount's actual definition. Packaged as a
  `xacro:macro` named `camera_mount` (params: `parent`, `*origin`), **not** a standalone
  robot — it can't be launched by itself, it has to be included and called from
  somewhere that supplies those two things:
  ```xml
  <xacro:include filename="$(find camera_mount_description)/urdf/camera_mount.urdf.xacro"/>
  <xacro:camera_mount parent="some_link">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:camera_mount>
  ```
  You should never need to edit anything inside this file to change *what it's attached
  to* — only the call site above changes. You only edit this file when the mount's own
  design changes (see "Re-syncing" below).
- **`urdf/camera_mount_standalone.urdf.xacro`** — a small **test-only** wrapper that calls
  the macro above with `parent="world"` (a fake, free-floating link this file invents)
  and an identity origin. This exists purely so `display.launch.py` and
  `camera_mount_bringup` have *something* to publish TF against before the mount is
  attached to a real robot. It is not part of the mount's design and isn't meant to be
  repurposed for a real robot — see "Attaching to a real robot" below.
- **`urdf/assets/`** — STL meshes referenced by the xacro.
- **`onshape_export/`** — raw, unedited onshape-to-robot output (`onshape.urdf`,
  `config.json`). See "What's `onshape_export/` for?" below.
- **`launch/display.launch.py`** — RViz preview (`camera_mount_standalone.urdf.xacro` +
  `robot_state_publisher` + `joint_state_publisher_gui` + RViz), no hardware involved.
- **`rviz/camera_mount.rviz`** — the saved RViz layout both `display.launch.py` and
  `camera_mount_bringup`'s launch file load.

## The `origin` you pass in is *not* "screw hole to screw hole"

It's the full transform from the parent link's frame to wherever `camera_mount_base_link`'s
own origin happens to sit — which is just whatever point OnShape's part frame landed on,
not necessarily the physical bolt holes. This is exactly the same situation as the D455's
`bottom_screw_frame`, which also isn't the actual screw location on this rear-mounted
design — it's `realsense2_description`'s own reference point, and the offset we solved for
it works anyway because it's a transform *to that frame*, not to the screws.

**Practical implication**: when you measure or CAD-solve the offset to attach this mount to
a real robot, the number you need is "parent link frame → `camera_mount_base_link`'s
origin," not "parent link frame → bolt holes." If those two points differ, fold that
difference into the measurement once; the macro call itself doesn't care either way.

## Attaching to a real robot (e.g. the Unitree G1)

Once you know which link on the target robot the mount bolts to, and the transform to
`camera_mount_base_link`'s origin (see above), the attachment happens **inside that
robot's own URDF/xacro file** — not in this package:

```xml
<!-- inside the target robot's own top-level xacro, alongside its existing links/joints -->
<xacro:include filename="$(find camera_mount_description)/urdf/camera_mount.urdf.xacro"/>

<xacro:camera_mount parent="head_link">
  <origin xyz="X Y Z" rpy="R P Y"/>
</xacro:camera_mount>
```

This makes the mount part of the same kinematic tree as the whole robot: moving the
robot's own head joints correctly carries the camera along in TF, and RViz shows the
mount sitting on the actual robot mesh — not floating alone in space.

Don't repurpose `camera_mount_standalone.urdf.xacro` for this by renaming its `world` link
to something like `head_link` — that only creates an empty dummy link with a
robot-sounding name, disconnected from the real robot's actual torso/arms/legs geometry.
It's useful for a quick, isolated "does this offset look plausible" check, but it is not
the same as real integration. `camera_mount_standalone.urdf.xacro` stays exactly what it
is: a disposable bench-test harness for this package alone.

Once real integration exists, `camera_mount_bringup`'s launch file should load the target
robot's URDF instead of `camera_mount_standalone.urdf.xacro`.

## What's `onshape_export/` for?

`onshape_export/onshape.urdf` is the **raw output of onshape-to-robot** — regenerated
wholesale on every OnShape export, and never hand-edited. It exists for exactly one
reason: it's the baseline you diff against next time you re-sync (see below) — without
it, you'd have no record of what the previous export looked like.

It is **never used at runtime**. Nothing in `launch/`, `CMakeLists.txt`, or
`camera_mount.urdf.xacro` references it, it isn't installed to the package's share
directory, and `xacro`/`robot_state_publisher` never touch it. Deleting it wouldn't break
anything at launch time — you'd just lose the ability to see what changed on the next
re-sync.

## Re-syncing after a fresh OnShape export

Whenever the OnShape design changes and gets re-exported (`onshape.urdf` overwritten,
mesh STLs regenerated):

1. **Check structure first, not just numbers.** List the `<link>` and `<joint>` names in
   the fresh `onshape.urdf` and compare against `camera_mount.urdf.xacro`. This is the
   only way to catch an added/removed link (e.g. a new OnShape mate-connector frame)
   before you start copying values — a pure "update the numbers" pass would silently
   miss it.
2. Copy the fresh `onshape.urdf` and any changed STLs over `onshape_export/onshape.urdf`
   and `urdf/assets/*.stl`.
3. Diff the fresh `onshape.urdf` against `urdf/camera_mount.urdf.xacro`, block by block:
   the three links (`camera_mount_base_link`, `camera_mount_yaw_link`,
   `camera_mount_pitch_link`) and the two joints (`yaw`, `pitch`). Copy every inertial/
   visual/collision `<origin>`, `<inertia>`, and `<mass>` value straight across — the
   structure of these blocks never changes between exports, only the numbers.
   - If only `camera_mount_base_link`'s own values and the `yaw` joint changed, while
     everything from `camera_mount_yaw_link` downward is untouched, that's usually a
     **re-basing** (OnShape's export root shifted, e.g. because of a new mate connector)
     rather than an actual design change — nothing physically moved, it's just described
     relative to a different reference point now.
4. **One deliberate difference to preserve**: inside `camera_mount_pitch_link`, the fresh
   export has a `"Part d455"` visual/collision block — do **not** copy that over. It stays
   dropped in the xacro; `xacro:sensor_d455` at the bottom of the file supplies the real
   mesh instead, correctly tied into TF (`camera_link` + friends) rather than floating
   disconnected from it.
5. **Check whether the D455 offset needs recomputing**: compare the `"Part d455"` visual's
   `<origin>` in the fresh export against what it was last time (noted in the xacro's own
   comment above `xacro:sensor_d455`). If those numbers changed, the camera moved relative
   to its holder in the CAD and the `xacro:sensor_d455` origin must be re-solved:
   ```
   T(pitch_link -> bottom_screw_frame) = T(pitch_link -> d455 mesh) . T(bottom_screw_frame -> d455 mesh)^-1
   ```
   using `realsense2_description`'s own `bottom_screw_frame -> mesh` offsets for the second
   term. If unchanged (the common case), leave `xacro:sensor_d455`'s origin as-is.
6. Rebuild and validate, in order:
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
7. Visually confirm: `ros2 launch camera_mount_description display.launch.py`.

If you test the export in a scratch workspace first (recommended — verify TF/meshes look
right before touching this package), the same seven steps apply there; port the same
value changes into this package's `camera_mount.urdf.xacro` once you're happy with the
result.

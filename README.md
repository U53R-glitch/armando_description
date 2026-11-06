# armando_description — Classwork 1 (Robotics Lab 2026)

ROS 2 Humble + Gazebo Harmonic description of **Armando**, a 4-DoF manipulator,
extended with:

- a **second camera mounted on the wrist** (eye-in-hand), attached to `crawer_base`
  and looking along the gripper approach axis (+z of `crawer_base`);
- a **C++ keyboard teleoperation node**, `armando_teleop`, that drives the arm
  through the `joint_trajectory_controller`.

## Changes with respect to the original package

| File | Change |
|---|---|
| `urdf/armando_wrist_camera.urdf.xacro` | **New.** Wrist camera: link, fixed joint on `crawer_base`, optical frame, Gazebo camera sensor (640x480, 30 Hz) |
| `urdf/arm.urdf.xacro` | Includes the wrist camera file |
| `launch/armando_gazebo.launch.py` | Two new `ros_gz_bridge` entries (wrist camera image and camera_info) |
| `src/armando_teleop.cpp` | **New.** Keyboard teleoperation node |
| `CMakeLists.txt`, `package.xml` | Build/install of the node and its dependencies (`rclcpp`, `sensor_msgs`, `trajectory_msgs`) |

## 1. From a clean machine to a running simulation

Requirements: Docker and the course Docker scripts (repository from Classwork 0).

### 1.1 Build the image and start the container (on the host)

```bash
git clone https://github.com/U53R-glitch/sarno_rl26.git
cd sarno_rl26/docker_scripts
./docker_build_image.sh armando_img
./docker_run_container.sh
```

In the wizard select `armando_img` and accept the default names: the container is
`armando_img_cont` and the host folder `~/armando_img_fold` is mounted as
`/home/user/ros2_ws/src` inside the container.

### 1.2 Get the package, install dependencies, build (inside the container)

```bash
cd /home/user/ros2_ws/src
git clone https://github.com/U53R-glitch/armando_description.git
cd /home/user/ros2_ws
sudo apt update                                   # Docker password: user
rosdep update
rosdep install --from-paths src --ignore-src -y   # installs Gazebo, ros_gz, ros2_control, ...
colcon build --symlink-install
source install/setup.bash
```

> Dependencies installed with `rosdep`/`apt` live in the container only: if the
> container is closed, repeat `sudo apt update`, `rosdep update` and `rosdep install`.

### 1.3 Launch the simulation (terminal 1)

```bash
ros2 launch armando_description armando_gazebo.launch.py
```

Every additional terminal must enter the same running container:

```bash
docker exec -it armando_img_cont bash     # on the host
cd /home/user/ros2_ws && source install/setup.bash
```

## 2. Validation of Task 1 — controllers

```bash
ros2 control list_controllers
# expected: joint_state_broadcaster and joint_trajectory_controller, both "active"

ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory "{
joint_names: ['j0', 'j1', 'j2', 'j3'],
points: [ { positions: [0.5, -0.3, 0.8, 0.0], time_from_start: { sec: 2, nanosec: 0 } } ]
}"
# expected: the arm reaches the configuration in about 2 s
```

## 3. Validation of Task 2 — wrist camera

```bash
ros2 topic list | grep camera
# expected:
#   /camera/camera_info          (fixed camera)
#   /camera/image_raw
#   /wrist_camera/camera_info    (wrist camera)
#   /wrist_camera/image_raw

ros2 topic hz /wrist_camera/image_raw
# ~30 Hz in simulated time (lower in wall time if the real-time factor is < 1)

ros2 topic echo /wrist_camera/camera_info --once | head -5
# frame_id: wrist_camera_optical_frame

ros2 run rqt_image_view rqt_image_view
# select /wrist_camera/image_raw: the view moves with the arm
```

Design notes:

- The fixed camera hangs off `base_link` (eye-to-hand); the wrist camera hangs off
  `crawer_base`, after joints j0–j3, so its pose depends on the joint configuration
  (eye-in-hand).
- `wrist_camera_link` looks along its +x axis; `rpy="0 -1.5708 0"` aligns it with
  the +z (approach) axis of `crawer_base`. The optical frame keeps the standard
  rotation (-pi/2, 0, -pi/2) required by image conventions (z forward, x right, y down).
- The Gazebo topic is `wrist_camera/image_raw`, so Gazebo publishes camera info on
  `/wrist_camera/camera_info` instead of `/camera_info`, which is already used by the
  fixed camera.

## 4. Validation of Task 3 — keyboard teleoperation

With the simulation running and `rqt_image_view` open on `/wrist_camera/image_raw`,
in a separate terminal (the node needs an interactive keyboard, so it is started
with `ros2 run`, not from a launch file):

```bash
ros2 run armando_description armando_teleop
```

| Key | Action |
|---|---|
| `q` / `a` | j0 + / − |
| `w` / `s` | j1 + / − |
| `e` / `d` | j2 + / − |
| `r` / `f` | j3 + / − |
| `h` | home (all joints to 0) |
| `x` or Ctrl+C | quit |

How it works:

- at start-up it reads `/joint_states` once (matching joints **by name**), so the
  first command starts from the current configuration;
- each key press moves one joint by 0.05 rad and publishes a single-point
  `trajectory_msgs/JointTrajectory` on `/joint_trajectory_controller/joint_trajectory`;
  each new command replaces the one in progress;
- targets are clamped to ±3.14 rad for j0 (URDF limits) and ±1.57 rad for j1–j3
  (safety margin against self-folding and table collisions);
- the motion time is `max(0.3 s, max|Δq| / 0.8 rad/s)`, so large moves (e.g. home)
  respect the URDF velocity limit of 1.0 rad/s.

Press keys one at a time: holding a key down triggers keyboard auto-repeat and
the target moves faster than the arm can follow.

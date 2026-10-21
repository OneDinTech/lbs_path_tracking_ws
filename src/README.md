# Path Tracker for a 3-Wheel Swerve Robot (ROS 2 Humble)

A look-ahead path tracker for a holonomic 3-wheel swerve drive robot.
The robot localizes with **FAST-LIO2** on a **Livox MID-360**, follows a
`nav_msgs/Path` on `/plan`, and drives its motors over **CAN**. A kinematic
simulation runs the same tracker with no hardware.

For competitions there is an interactive **path generator**: type a goal
(x, y, theta) measured from the robot center, and the robot drives there.

---

## 1. Requirements

* Ubuntu 22.04 + ROS 2 Humble
* Built and sourced: [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2),
  [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2),
  [FAST_LIO (ROS 2)](https://github.com/hku-mars/FAST_LIO) with a `mid360.yaml` config
* A SocketCAN interface (`can0`) for the motors (real robot only)
* For the simulation tools: `python3-numpy`, `python3-yaml`, `python3-matplotlib`

## 2. Build

Clone this repository **as the `src` folder** of a workspace:

```bash
mkdir -p ~/path_tracking_ws
git clone <this-repo-url> ~/path_tracking_ws/src
cd ~/path_tracking_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build
```

Each new terminal (Livox and FAST-LIO first, then this workspace):

```bash
source ~/ws_livox/install/setup.bash        # your Livox + FAST-LIO workspace
source ~/path_tracking_ws/install/setup.bash
```

---

## 3. Run: competition (real robot + path generator)

**Once per boot** (adapt the interface names and IP to your setup):

```bash
sudo ip link set can0 up type can bitrate 1000000           # motors
sudo ip addr add 192.168.1.5/24 dev <lidar_ethernet_iface>  # MID-360 network
```

**Terminal 1: start the robot**

```bash
ros2 launch robot_bringup bringup.launch.py mode:=auto
```

Yes, this already includes the Livox driver and FAST-LIO. It starts in stages:

| Time | What starts |
|------|-------------|
| 0 s | robot model (URDF), static TFs, joystick |
| 3 s | Livox driver (`msg_MID360_launch.py`) |
| 6 s | FAST-LIO (`mapping.launch.py`, `mid360.yaml`). **Keep the robot still for about 3 s** |
| 9 s | `path_tracker` + `robot_controller` (**motors live**) + CSV recorder |

No joystick? Add `use_joy:=false`.

**Terminal 2: give goals**

```bash
ros2 run controller path_generator
```

```
New goal
  x     [m]  : 1.0
  y     [m]  : 0.5
  theta [deg]: 90
  path published on /plan
```

| Input | Unit | Positive direction |
|-------|------|--------------------|
| `x` | m | forward |
| `y` | m | **right** |
| `theta` | deg | **clockwise** (negative = counter clockwise), range -180 to 180 |

* Every goal is measured from the **robot center at the moment you press
  Enter**, wherever the robot is on the map. The origin always moves with the robot.
* The path is a straight line with a pose every 0.05 m (like `path_planner_v2.py`),
  each pose with its own heading, interpolated from the start to the goal heading.
* `x = 0, y = 0` with a theta: the robot turns in place.
* Empty input counts as 0. A new goal replaces the current one. `q` quits.
* Turning more than 180 degrees: send it as two goals.
* Final accuracy is set by `distance_goal_tolerance` and `yaw_goal_tolerance`
  (see section 6).

## 4. Run: simulation (no hardware)

```bash
ros2 launch robot_bringup sim.launch.py          # terminal 1 (opens RViz)
ros2 run controller path_generator               # terminal 2
```

The sim uses the same tracker, the same tuning and the same motor limits as
the real robot. It does not model lidar noise, wheel slip or motor lag.

In RViz set **Fixed Frame** to `map`, then add **RobotModel**
(topic `/robot_description`), **TF**, and **Path** on `/plan` and `/traveled_path`.

**Predefined shapes** (circle, square, triangle, star, custom waypoints)
instead of single goals:

```bash
cd ~/path_tracking_ws/src/simulation
python3 path_planner_v2.py        # pick shape + interpolation in the menu
```

## 5. Other launch options

`bringup.launch.py mode:=<mode>`

| Mode | What runs |
|------|-----------|
| `auto` (default) | localization + path tracker. Motors live |
| `sensors_only` | localization only. **Motors stay off** (safe TF / odometry test) |
| `manual_drive_with_sensors` | joystick driving + localization |
| `manual_drive_no_sensors` | joystick driving only |

Extra arguments: `use_joy:=false`, `use_rviz:=true`, `record:=false`.
Simulation: `sim.launch.py mode:=manual` drives the sim with the joystick.

**Recorded data.** In `auto` (and in the sim) every run is saved to
`src/simulation/run_<date>_<time>.csv`. Plot the newest run with:

```bash
cd ~/path_tracking_ws/src/simulation && python3 plot_run.py
```

If you cloned somewhere else than `~/path_tracking_ws/src`, pass
`recorder_script:=/full/path/to/simulation/data_recorder.py` or `record:=false`.

---

## 6. Changing parameters

1. Edit the YAML file in `src/` (table below).
2. Rebuild so the file is copied to `install/`: `colcon build`
3. Relaunch.

To try a value without restarting (lost on the next launch):

```bash
ros2 param set /path_tracker kp_heading 1.2
```

| File | Controls | Most used parameters |
|------|----------|----------------------|
| `controller/src/path_tracker/path_tracker_config.yaml` | Tracker tuning, **shared by the real robot and the sim** (`/**` block) | `desired_linear_velocity`, `kp_heading`, `lookahead_time`, `max_linear_acceleration`, `maximum_angular_velocity`, `corner_velocity`, `distance_goal_tolerance`, `yaw_goal_tolerance` |
| `controller/src/kinematic/robot_controller_config.yaml` | Motor driver: CAN ids, wheel geometry, speed / acceleration limits, safety timeout (also used by the sim) | `speed_ids`, `steer_ids`, `robot_l`, `wheel_r`, `max_linear_velocity`, `wheel_speed_max`, `invert_drive`, `cmd_timeout` |
| `controller/src/ps4/manual_config.yaml` | Joystick speeds, axes, buttons | `linear_speed`, `angular_speed`, `axis_*`, `btn_*` |
| `simulation/path_config_v2.yaml` | Shapes and waypoints for `path_planner_v2.py` (no rebuild needed) | `shapes`, `waypoints`, `point_spacing` |

**Using a different robot?** Also check the URDF
(`robot_description/urdf/robot.xacro.urdf`) and the two static transforms
(`map -> camera_init`, `body -> base_footprint`) in
`robot_bringup/launch/bringup.launch.py`. They hold the lidar mounting
position.

---

## 7. Repository structure

```
src/
├── README.md
├── controller/                     C++ package: tracker, path generator, motor driver
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/controller/         headers (tracker math, kinematics, CAN, packing)
│   └── src/
│       ├── main/
│       │   ├── path_tracker.cpp        real robot follower: TF pose + /plan -> /cmd_vel
│       │   ├── path_tracker_sim.cpp    kinematic sim with the same tracker + motor limits
│       │   └── path_generator.cpp      interactive goal (x, y, theta) -> /plan
│       ├── path_tracker/
│       │   ├── Path_Tracker.cpp        tracking math (look-ahead, speed, heading, corners)
│       │   └── path_tracker_config.yaml
│       ├── kinematic/
│       │   ├── robot_controller.cpp    /cmd_vel -> swerve IK -> CAN motors
│       │   ├── Swervedrive_Kinematic.cpp
│       │   └── robot_controller_config.yaml
│       ├── path_generator/Path_Generation.cpp   path helper functions (library)
│       ├── Localizer/odom_wheel.cpp    dead wheel + IMU odometry (fallback, unused by default)
│       ├── calibrator/calibrate.cpp    wheel / steering calibration tool
│       ├── ps4/                        PS4 joystick -> /cmd_vel_manual (+ YAML)
│       ├── driver/driver.cpp           motor CAN frame protocol
│       └── lib/                        SocketCAN wrapper, byte packing
├── robot_bringup/                  launch files
│   └── launch/
│       ├── bringup.launch.py           real robot: model, TF, Livox, FAST-LIO, tracker, motors
│       ├── sim.launch.py               simulation + RViz
│       └── description.launch.py       robot model + joint sliders + RViz (no motors)
├── robot_description/              URDF of the robot
│   └── urdf/robot.xacro.urdf
└── simulation/                     Python tools (not a ROS package)
    ├── path_planner_v2.py          shape planner, full path anchored to the robot pose
    ├── path_planner.py             shape planner, one leg at a time (stops at each vertex)
    ├── path_config_v2.yaml         shapes / waypoints for the planners
    ├── data_recorder.py            records a run to CSV (started by the launch files)
    ├── plot_run.py                 plots a recorded run (9 graphs)
    └── ...                         older prototypes and debug tools
```

### Topics

| Topic | Type | From -> To |
|-------|------|------------|
| `/plan` | `nav_msgs/Path` (frame `map`) | path_generator / planner -> path_tracker |
| `/cmd_vel` | `geometry_msgs/Twist` (body frame) | path_tracker -> robot_controller |
| `/cmd_vel_manual` | `geometry_msgs/Twist` | ps4 -> robot_controller |
| `/joint_states` | `sensor_msgs/JointState` | robot_controller or sim -> RViz |
| TF `map -> base_footprint` | | FAST-LIO + static TFs (real) or the sim |

## Safety

In `auto` and the manual modes the motors are live. Clear the area first.
`robot_controller` stops the robot if no command arrives within
`cmd_timeout` (0.3 s), and `path_tracker` stops if the pose is older than
`pose_timeout` (0.25 s). Use `sensors_only` to test without driving.

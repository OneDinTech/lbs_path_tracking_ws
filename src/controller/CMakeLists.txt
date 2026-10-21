cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# tf2: path_tracker looks up map -> base_footprint (FAST-LIO via the
# static bridge). odom_wheel broadcasts odom -> base_link in its own mode
# (kept buildable but not used by the FAST-LIO-only bringup).
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# std_srvs: odom_wheel's ~/reset Trigger service
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ─── shared library ─────────────────────────────────────────
# Sources now live in subfolders under src/ (driver/, lib/, kinematic/,
# path_generator/, path_tracker/). Headers are still under
# include/controller, so the #include "can.hpp" style paths are unchanged.
# odrive.cpp / vesc.cpp are NOT listed: those source files do not exist
# (only their headers do) and no node uses them. Add them back here only
# once src/driver/odrive.cpp and src/driver/vesc.cpp are written.
# STATIC on purpose: the library is not installed, so a shared build would
# leave the installed executables unable to find it at runtime.
add_library(controller_lib STATIC
src/lib/can.cpp
src/lib/packer.cpp
src/driver/driver.cpp
src/path_tracker/Path_Tracker.cpp
src/kinematic/Swervedrive_Kinematic.cpp
src/path_generator/Path_Generation.cpp
)

target_include_directories(controller_lib PUBLIC
include/controller
)

ament_target_dependencies(controller_lib
rclcpp
nav_msgs
geometry_msgs
std_msgs
sensor_msgs
)

# ─── real robot executable: path_tracker (TF pose follower) ─
# Reads map -> base_footprint straight from TF (FAST-LIO's own
# camera_init -> body plus the static bridges in the bringup launch),
# follows a path in the map frame, and publishes /joint_states for the
# RViz wheel animation. No odometry topic, no EKF. Needs tf2 for the
# lookup and sensor_msgs for JointState.
# path_tracker is now a PURE PATH FOLLOWER: reads the pose from TF,
# runs path tracker, publishes /cmd_vel. No CAN, no kinematics, so it
# needs only the path tracker math (compiled straight from Path_Tracker.cpp via
# controller_lib) and the ROS message/TF deps. No pthread, no std_msgs.
add_executable(path_tracker
src/main/path_tracker.cpp
)

target_include_directories(path_tracker PRIVATE
include/controller
)

target_link_libraries(path_tracker
controller_lib
)

ament_target_dependencies(path_tracker
rclcpp
nav_msgs
geometry_msgs
rcl_interfaces
tf2
tf2_ros
)

# ─── path_generator: interactive goal -> /plan ──────────────
# Asks for x (fwd), y (right), theta (deg, CW+) relative to the robot
# center, builds a straight path from the current pose, publishes /plan.
# Pure ROS node, no CAN and no tracker math, so no controller_lib.
add_executable(path_generator
src/main/path_generator.cpp
)

ament_target_dependencies(path_generator
rclcpp
nav_msgs
geometry_msgs
tf2
tf2_ros
)

# ─── robot_controller: the swerve MOTOR DRIVER ──────────────
# Subscribes /cmd_vel, runs IK, drives the CAN motors, publishes
# /joint_states for RViz. The ONLY node that drives the autonomous
# motors. It listens on both /cmd_vel (auto) and /cmd_vel_manual (manual).
add_executable(robot_controller
src/kinematic/robot_controller.cpp
)

target_include_directories(robot_controller PRIVATE
include/controller
)

target_link_libraries(robot_controller
controller_lib
pthread
)

ament_target_dependencies(robot_controller
rclcpp
geometry_msgs
sensor_msgs
)

# ─── ODOMETRY: dead wheels + gyro ───────────────────────────
# Reads the two passive encoder wheels off CAN, integrates them with
# gyro yaw from /imu/data, and owns odom -> base_link.
# NOT used by the FAST-LIO-only bringup. Kept buildable as a fallback.
# Needs pthread for its RX thread, tf2_ros to broadcast, std_srvs for
# the reset service, sensor_msgs for the IMU subscription.
add_executable(odom_wheel
src/Localizer/odom_wheel.cpp
)

target_include_directories(odom_wheel PRIVATE
include/controller
)

target_link_libraries(odom_wheel
controller_lib
pthread
)

ament_target_dependencies(odom_wheel
rclcpp
nav_msgs
sensor_msgs
geometry_msgs
std_srvs
tf2
tf2_ros
)

# ─── simulation executable ──────────────────────────────────
# Kinematic sim with perfect odometry: runs the same path tracker as
# path_tracker, integrates the pose from the wheel commands, and
# publishes map -> base_footprint TF + /joint_states so RViz animates
# the robot driving a /plan path. No CAN, so no controller_lib link.
add_executable(path_tracker_sim
src/main/path_tracker_sim.cpp
)

target_include_directories(path_tracker_sim PRIVATE
include/controller
)

target_link_libraries(path_tracker_sim
controller_lib
)

ament_target_dependencies(path_tracker_sim
rclcpp
nav_msgs
geometry_msgs
sensor_msgs
std_msgs
tf2_ros
)

# ─── calibration executable ─────────────────────────────────
add_executable(calibrate
src/calibrator/calibrate.cpp
)

target_include_directories(calibrate PRIVATE
include/controller
)

target_link_libraries(calibrate
controller_lib
pthread
)

# ─── PS4 teleop ─────────────────────────────────────────────
add_executable(ps4
src/ps4/ps4.cpp
)

target_include_directories(ps4 PRIVATE
include/controller
)

ament_target_dependencies(ps4
rclcpp
geometry_msgs
sensor_msgs
rcl_interfaces
)

# ─── install all ────────────────────────────────────────────
install(TARGETS
  path_tracker
  path_generator
  robot_controller
  odom_wheel
  path_tracker_sim
  ps4
  calibrate
DESTINATION lib/${PROJECT_NAME}
)

# The path tracker tuning YAML. Installed so the launch files can resolve it at
# runtime (launch resolves against the install space, not src). With
# colcon build --symlink-install, edits to the source YAML take effect on
# the next launch without rebuilding.
# path tracker tuning YAML. The launch looks for it at
# share/controller/path_tracker/path_tracker_config.yaml. Install just
# the file (not the .cpp/.hpp sources that also live in src/path_tracker).
install(FILES src/path_tracker/path_tracker_config.yaml
DESTINATION share/${PROJECT_NAME}/path_tracker
)

# Manual-drive + teleop tuning YAML. The launch looks for it at
# share/controller/config/manual_config.yaml, so install just that file
# into a config/ subfolder (not the whole src/ps4 source dir).
install(FILES src/ps4/manual_config.yaml
DESTINATION share/${PROJECT_NAME}/config
)

# Autonomous motor-driver YAML. The launch looks for it at
# share/controller/kinematic/robot_controller_config.yaml.
install(FILES src/kinematic/robot_controller_config.yaml
DESTINATION share/${PROJECT_NAME}/kinematic
)

ament_package()
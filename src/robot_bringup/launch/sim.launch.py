#!/usr/bin/env python3
# =====================================================================
# sim.launch.py -- KINEMATIC SIMULATION of the swerve robot, perfect
# odometry. No lidar, no CAN, no motors, no Gazebo.
#
# ONE ARGUMENT: mode.
#   auto   (default) -- path_tracker_sim follows the /plan path with path tracker.
#                       Send goals with path_generator (path starts at the
#                       robot center, same as on the real robot).
#   manual           -- drive the sim robot with the joystick
#                       (/cmd_vel_manual), no path needed.
#
# Brings up:
#   - robot_state_publisher  (URDF: base_footprint -> ... -> wheels)
#   - path_tracker_sim                 (path tracker follower OR manual, + perfect-pose sim;
#                             owns map -> base_footprint and /joint_states)
#   - joy_node + ps4         (manual mode only)
#   - RViz
#
# path_tracker_sim owns map -> base_footprint, so DO NOT run the hardware bringup's
# static bridges or FAST-LIO alongside this.
#
# Run:
#   ros2 launch robot_bringup sim.launch.py                  # auto
#   ros2 launch robot_bringup sim.launch.py mode:=manual     # joystick
#   (auto mode) then, in another terminal:
#   ros2 run controller path_generator
# =====================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONTROLLER_PKG = "controller"
PT_YAML_REL = os.path.join("path_tracker", "path_tracker_config.yaml")
DESC_PKG = "robot_description"
XACRO_REL = os.path.join("urdf", "robot.xacro.urdf")
RVIZ_REL = os.path.join("rviz", "robot.rviz")


def generate_launch_description():
    desc_share = get_package_share_directory(DESC_PKG)
    xacro_path = os.path.join(desc_share, XACRO_REL)
    rviz_path = os.path.join(desc_share, RVIZ_REL)
    pt_yaml = os.path.join(
        get_package_share_directory(CONTROLLER_PKG), PT_YAML_REL)
    # Same motor-driver limits the real robot_controller uses, so the sim
    # clamps / slew-limits / desaturates exactly like the hardware.
    rc_yaml = os.path.join(
        get_package_share_directory(CONTROLLER_PKG), 'kinematic', 'robot_controller_config.yaml')
    manual_yaml = os.path.join(
        get_package_share_directory(CONTROLLER_PKG), 'config', 'manual_config.yaml')

    mode = LaunchConfiguration("mode")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]), value_type=str)

    # manual_mode param passed to path_tracker_sim: true when mode:=manual
    manual_flag = PythonExpression(["'", mode, "' == 'manual'"])

    # joystick runs only in manual mode
    is_manual = IfCondition(PythonExpression(["'", mode, "' == 'manual'"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="auto",
            description="auto = follow /plan with path tracker, "
                        "manual = drive with the joystick"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "joy_dev_id", default_value="0",
            description="joystick device id, /dev/input/js<N> (manual mode)"),

        # robot model -> TF below base_footprint
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # the sim: path tracker follower OR manual drive + perfect pose. Owns
        # map -> base_footprint and /joint_states. Loads path tracker tuning from
        # the shared YAML; manual_mode is set from the launch mode.
        Node(
            package="controller",
            executable="path_tracker_sim",
            name="path_tracker_sim",
            output="screen",
            parameters=[
                pt_yaml,
                rc_yaml,
                {"manual_mode": ParameterValue(manual_flag, value_type=bool)},
            ],
        ),

        # NOTE: no joint_state_publisher_gui. path_tracker_sim publishes /joint_states.

        # ---- joystick chain (manual mode only) ----
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "device_id": LaunchConfiguration("joy_dev_id"),
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
            }],
            condition=is_manual,
        ),
        Node(
            package="controller",
            executable="ps4",
            name="dynamic_teleop",   # must match the YAML section name
            output="screen",
            parameters=[manual_yaml],
            condition=is_manual,
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=(["-d", rviz_path] if os.path.exists(rviz_path) else []),
            condition=IfCondition(use_rviz),
        ),
    ])

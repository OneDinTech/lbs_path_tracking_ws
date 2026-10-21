#!/usr/bin/env python3
"""
description.launch.py

Launches joint_state_publisher + robot_state_publisher + RViz2 for the
swervedrive_robot xacro, plus the two static transforms that connect
FAST-LIO's tree (camera_init / body) to the robot tree.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = "robot_description"
XACRO_RELATIVE_PATH = os.path.join("urdf", "robot.xacro.urdf")
RVIZ_RELATIVE_PATH = os.path.join("rviz", "robot.rviz")


def generate_launch_description():
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    xacro_path = os.path.join(pkg_share, XACRO_RELATIVE_PATH)
    rviz_path = os.path.join(pkg_share, RVIZ_RELATIVE_PATH)

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2.",
    )
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description_content = Command(["xacro ", xacro_path])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    # --- static transforms connecting FAST-LIO tree to robot tree ---
    # map -> camera_init : places map at the robot's start center (on ground)
    map_to_camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_camera_init",
        output="screen",
        arguments=["0.109", "-0.16329", "0.11912", "0", "0", "0",
                   "map", "camera_init"],
    )

    # body -> base_footprint : bridge FAST-LIO IMU frame into robot tree
    body_to_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_footprint",
        output="screen",
        arguments=["-0.109", "0.16329", "-0.11912", "0", "0", "0",
                   "body", "base_footprint"],
    )

    rviz_args = ["-d", rviz_path] if os.path.exists(rviz_path) else []
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=rviz_args,
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            robot_state_publisher_node,
            joint_state_publisher_node,
            map_to_camera_init,
            body_to_base_footprint,
            rviz_node,
        ]
    )
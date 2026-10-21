# =====================================================================
# bringup.launch.py -- full robot bring-up, staged, single MODE switch.
#
# ONE ARGUMENT: mode. Four values.
#
#   manual_drive_no_sensors
#     joystick + manual_drive ONLY. No TF bridges, no lidar, no FAST-LIO.
#     Bare teleop. Motors live (driven by the stick).
#
#   manual_drive_with_sensors
#     joystick + manual_drive + full localization (static TF + livox +
#     FAST-LIO). Drive by stick while odometry runs, e.g. to record a
#     path or watch drift. Motors live.
#
#   sensors_only
#     full localization, MOTORS STAY DEAD. Nothing writes CAN. Use this
#     to test the TF tree and odometry in RViz safely.
#
#   auto
#     full localization + path_tracker. Fully autonomous path
#     following. Motors live.
#
# ---------------------------------------------------------------------
# LOCALIZATION (runs in every mode EXCEPT manual_drive_no_sensors):
#
#   TF tree:
#     map -> camera_init   static (this file)
#     camera_init -> body  dynamic (FAST-LIO's own broadcast, ~10 Hz)
#     body -> base_footprint static (this file, the sensor bridge)
#     base_footprint -> base_link -> {livox_frame, wheels}  (URDF, from
#       robot_state_publisher, now started by THIS file in stage 0)
#
#   path_tracker reads map -> base_footprint from TF.
#   path_planner_v2.py must publish in "map" (frame_id in path_config_v2.yaml).
#
# Exactly one publisher per TF edge:
#   - FAST-LIO owns camera_init -> body
#   - this file owns map -> camera_init and body -> base_footprint
#   - robot_state_publisher owns base_footprint and below (started here)
# ---------------------------------------------------------------------
# STAGES  (TimerAction, so ordering is enforced)
#
#   t=0s   static TFs (localization modes), joystick (manual modes)
#   t=3s   livox driver          (localization modes)
#   t=6s   FAST-LIO              (localization modes)
#            KEEP THE ROBOT STILL ~3s after this: it estimates IMU bias.
#   t=9s   path_tracker (auto only). Autonomous motors live here.
#
# manual_drive (both manual modes) starts IMMEDIATELY at t=0, since it
# needs neither TF nor FAST-LIO. sensors_only runs no motor node.
# ---------------------------------------------------------------------
#
# USAGE
#   ros2 launch robot_bringup bringup.launch.py mode:=auto
#   ros2 launch robot_bringup bringup.launch.py mode:=sensors_only
#   ros2 launch robot_bringup bringup.launch.py mode:=manual_drive_with_sensors
#   ros2 launch robot_bringup bringup.launch.py mode:=manual_drive_no_sensors
#   ros2 launch robot_bringup bringup.launch.py mode:=auto use_joy:=false
#
# PREREQUISITES each boot (the launch cannot do these):
#   sudo ip addr flush dev enp3s0
#   sudo ip addr add 192.168.1.5/24 dev enp3s0
#   sudo ip link set can0 up type can bitrate 1000000
# (the network/CAN are only needed for the modes that use lidar/motors)
# =====================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction, GroupAction, LogInfo, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PythonExpression,
                                  Command)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _optional_share(pkg):
    """Resolve a package share dir, or None if the package is not on
    AMENT_PREFIX_PATH. Runs at module load, before conditions evaluate.
    The lidar packages live in ws_livox, which may not be sourced."""
    try:
        return get_package_share_directory(pkg)
    except Exception:
        return None


def generate_launch_description():

    livox_share    = _optional_share('livox_ros_driver2')
    fast_lio_share = _optional_share('fast_lio')
    controller_share = get_package_share_directory('controller')
    desc_share = get_package_share_directory('robot_description')
    xacro_path = os.path.join(desc_share, 'urdf', 'robot.xacro.urdf')
    pt_yaml = os.path.join(
        controller_share, 'path_tracker', 'path_tracker_config.yaml')
    manual_yaml = os.path.join(
        controller_share, 'config', 'manual_config.yaml')
    rc_yaml = os.path.join(
        controller_share, 'kinematic', 'robot_controller_config.yaml')

    mode    = LaunchConfiguration('mode')
    use_joy = LaunchConfiguration('use_joy')
    record  = LaunchConfiguration('record')
    recorder_script = LaunchConfiguration('recorder_script')

    args = [
        DeclareLaunchArgument(
            'mode', default_value='auto',
            description='manual_drive_no_sensors | manual_drive_with_sensors '
                        '| sensors_only | auto'),
        DeclareLaunchArgument(
            'use_joy', default_value='true',
            description='run joy_node + ps4. Needed in both manual modes; '
                        'in auto it is the manual override (press y). '
                        'Irrelevant in sensors_only.'),
        DeclareLaunchArgument(
            'joy_dev_id', default_value='0',
            description='joystick device id, /dev/input/js<N>'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='open RViz'),
        DeclareLaunchArgument(
            'record', default_value='true',
            description='run data_recorder to log an auto run to CSV'),
        DeclareLaunchArgument(
            'recorder_script',
            default_value=os.path.expanduser(
                '~/path_tracking_ws/src/simulation/data_recorder.py'),
            description='path to data_recorder.py'),
    ]

    # ---- mode predicates -------------------------------------------
    # localization (TF + livox + FAST-LIO) runs in every mode except
    # the bare teleop one.
    want_localization = IfCondition(PythonExpression(
        ["'", mode, "' != 'manual_drive_no_sensors'"]))

    # manual_drive runs in both manual modes.
    want_manual = IfCondition(PythonExpression(
        ["'", mode, "' == 'manual_drive_no_sensors' or "
         "'", mode, "' == 'manual_drive_with_sensors'"]))

    # path_tracker runs only in auto.
    want_auto = IfCondition(PythonExpression(["'", mode, "' == 'auto'"]))

    # joystick runs when requested AND the mode uses a stick (both manual
    # modes, or auto as the override). Not in sensors_only.
    want_joy = IfCondition(PythonExpression(
        ["'", use_joy, "' == 'true' and '", mode, "' != 'sensors_only'"]))

    # =================================================================
    # STAGE 0: robot model + static TFs + joystick
    # =================================================================

    # robot_state_publisher: the URDF -> base_footprint -> ... -> wheels.
    # Brought up here so `bringup` is self-contained; no separate
    # description.launch.py needed.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_path = os.path.join(desc_share, 'rviz', 'robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=(['-d', rviz_path] if os.path.exists(rviz_path) else []),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # map -> camera_init. Places map at the robot's start centre so map
    # and base_footprint coincide at t=0. Numbers = sensor (IMU/body)
    # position in base_footprint:
    #   base_footprint->base_link (0,0,0.045) + base_link->livox
    #   (0.12,-0.14,0.03) + livox->IMU extrinsic_T (-0.011,-0.02329,0.04412)
    #   = (0.109, -0.16329, 0.11912). Recompute BOTH statics if the URDF
    #   lidar joint or the FAST-LIO extrinsic_T changes.
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init',
        arguments=['0.109', '-0.16329', '0.11912', '0.0', '0.0', '0.0',
                   'map', 'camera_init'],
        output='screen',
        condition=want_localization,
    )

    # body -> base_footprint. Inverse of the above, bridging FAST-LIO's
    # IMU frame into the robot tree.
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base_footprint',
        arguments=['-0.109', '0.16329', '-0.11912', '0.0', '0.0', '0.0',
                   'body', 'base_footprint'],
        output='screen',
        condition=want_localization,
    )

    # ---- joystick chain --------------------------------------------
    # autorepeat_rate: without it joy_node publishes only on change, so
    # holding the stick still produces no messages and manual_drive's
    # watchdog stops the robot mid manoeuvre.
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('joy_dev_id'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        condition=want_joy,
    )

    # CHECK which topic ps4.cpp publishes. Both path_tracker and
    # manual_drive listen to /cmd_vel_manual. If ps4 publishes /cmd_vel,
    # remap here:  remappings=[('cmd_vel', 'cmd_vel_manual')],
    ps4_node = Node(
        package='controller',
        executable='ps4',
        name='dynamic_teleop',   # must match the YAML section name
        output='screen',
        parameters=[manual_yaml],
        condition=want_joy,
    )

    stage0 = GroupAction([
        LogInfo(msg='[bringup] stage 0: model + static TFs + joystick'),
        rsp_node,
        rviz_node,
        static_tf_map,
        static_tf_base,
        joy_node,
        ps4_node,
    ])

    # =================================================================
    # STAGE 1: lidar driver (localization modes)
    # =================================================================
    # msg_MID360, not rviz_MID360: FAST-LIO needs xfer_format 1
    # (CustomMsg) for the per point timestamps it deskews with.
    if livox_share is not None:
        stage1_actions = [
            LogInfo(msg='[bringup] stage 1: livox driver'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(livox_share, 'launch_ROS2',
                                 'msg_MID360_launch.py')),
                condition=want_localization,
            ),
        ]
    else:
        stage1_actions = [LogInfo(
            msg='[bringup] livox_ros_driver2 not found, skipping. '
                'Source ~/ws_livox/install/setup.bash if you wanted the lidar.')]

    stage1 = TimerAction(period=3.0, actions=stage1_actions)

    # =================================================================
    # STAGE 2: FAST-LIO (localization modes)
    # =================================================================
    # FAST-LIO broadcasts camera_init -> body itself. That IS the moving
    # transform the tree hangs on. Do not add another publisher for it.
    if fast_lio_share is not None:
        stage2_actions = [
            LogInfo(msg='[bringup] stage 2: FAST-LIO. KEEP THE ROBOT STILL ~3s'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')),
                launch_arguments={'config_file': 'mid360.yaml'}.items(),
                condition=want_localization,
            ),
        ]
    else:
        stage2_actions = [LogInfo(msg='[bringup] fast_lio not found, skipping.')]

    stage2 = TimerAction(period=6.0, actions=stage2_actions)

    # =================================================================
    # STAGE 3: motor control
    # =================================================================
    # MOTORS GO LIVE HERE. In sensors_only NOTHING in this stage runs.
    # Only one of the two ever runs: they write the same CAN ids.

    # path_tracker is now a PATH FOLLOWER: publishes /cmd_vel only.
    path_tracker = Node(
        package='controller',
        executable='path_tracker',
        name='path_tracker',
        output='screen',
        parameters=[pt_yaml],
        condition=want_auto,
    )

    # robot_controller is the SINGLE motor driver. It listens on BOTH
    # /cmd_vel (auto, from path_tracker) and /cmd_vel_manual (manual,
    # from ps4). We launch it in whichever mode is active. In auto it
    # comes up at t=9 with path_tracker; in manual it comes up at t=0
    # with the joystick. Same executable, same config, same node name.
    robot_controller_auto = Node(
        package='controller',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[rc_yaml],
        condition=want_auto,
    )

    robot_controller_manual = Node(
        package='controller',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[rc_yaml],
        condition=want_manual,
    )

    # In manual mode the motor driver + joystick are live at t=0: no TF
    # and no FAST-LIO to wait for. In manual_drive_no_sensors the stick is
    # live at once; in manual_drive_with_sensors it drives while
    # localization spins up.
    stage_manual = GroupAction([
        LogInfo(msg='[bringup] MOTORS LIVE (manual). CLEAR THE AREA'),
        robot_controller_manual,
    ])

    # path_tracker still starts LAST (t=9) so it never follows a
    # path before FAST-LIO's TF exists. Only runs in auto.
    # CSV recorder: only in auto (needs a /plan). Self-starts on the
    # first /plan and stops at goal; safe to launch alongside the
    # controller.
    recorder = ExecuteProcess(
        cmd=['python3', recorder_script],
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", record, "' == 'true' and '", mode, "' == 'auto'"])),
    )

    stage3 = TimerAction(period=9.0, actions=[
        LogInfo(msg='[bringup] stage 3: autonomous control coming up, '
                    'CLEAR THE AREA'),
        path_tracker,
        robot_controller_auto,
        recorder,
    ])

    return LaunchDescription(
        args + [stage_manual, stage0, stage1, stage2, stage3])
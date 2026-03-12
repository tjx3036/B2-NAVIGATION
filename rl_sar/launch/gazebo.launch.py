# Copyright (c) 2024-2025 Ziqi Fan (Modified by Sanghyun Kim)
# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

# 设置环境变量以支持 Wayland/X11
os.environ['QT_QPA_PLATFORM'] = 'xcb'  # 强制使用 X11
os.environ['GDK_BACKEND'] = 'x11'

# PedestrianSFMPlugin 路径（移动行人插件）
_SFM_PLUGIN_DIR = os.path.expanduser("~/222/world/gazebo_sfm_plugin-master")
if os.path.exists(_SFM_PLUGIN_DIR):
    _current = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    os.environ["GAZEBO_PLUGIN_PATH"] = f"{_SFM_PLUGIN_DIR}:{_current}" if _current else _SFM_PLUGIN_DIR


def generate_launch_description():
    rname = LaunchConfiguration("rname")
    world_file = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    # 使用空世界，不加载复杂的世界模型
    wname = "empty"  # 使用空世界
    robot_name = ParameterValue(Command(["echo -n ", rname]), value_type=str)
    ros_namespace = ParameterValue(Command(["echo -n ", "/", rname, "_gazebo"]), value_type=str)
    gazebo_model_name = ParameterValue(Command(["echo -n ", rname, "_gazebo"]), value_type=str)

    robot_description = ParameterValue(
        Command([
            "xacro ",
            Command(["echo -n ", Command(["ros2 pkg prefix ", rname, "_description"])]),
            "/share/", rname, "_description/xacro/robot.xacro"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "verbose": "true",
            "pause": "false",
            "gui": "true",  # 显式启用 GUI
            "world": world_file,
        }.items(),
    )
    spawn_entity = TimerAction(
        period=15.0,  # 延迟 15 秒启动，等待 Gazebo 和插件完全初始化
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic", "/robot_description",
                    "-entity", "robot_model",
                    "-x", spawn_x,   # X坐标（前后方向）
                    "-y", spawn_y,   # Y坐标（左右方向）
                    "-z", spawn_z,   # Z坐标（高度）
                    "-Y", spawn_yaw, # 绕Z轴旋转角度（yaw，单位：弧度）
                    "-timeout", "60",  # 超时时间
                ],
                output="screen",
            )
        ]
    )

    joint_state_broadcaster_node = TimerAction(
        period=6.0,  # 延迟 6 秒启动，等待 spawn_entity 完成
        actions=[
            Node(
                package="controller_manager",
                executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
                arguments=["joint_state_broadcaster"],
                output="screen",
            )
        ]
    )

    robot_joint_controller_node = Node(
        package="controller_manager",
        executable='spawner.py' if os.environ.get('ROS_DISTRO', '') == 'foxy' else 'spawner',
        arguments=["robot_joint_controller"],
        output="screen",
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 0.0,
        }],
    )

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
        }],
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(
                    [rname, '_description']
                ),
                'launch',
                'display.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.expanduser("~/222/world/0404.world"),
            description="World file path. 默认 ~/222/world/0404.world",
        ),
        DeclareLaunchArgument(
            "rname",
            description="Robot name (e.g., a1, go2, b2)",
            default_value=TextSubstitution(text="b2"),  # 默认使用 b2
        ),
        DeclareLaunchArgument(
            "spawn_x",
            description="Robot spawn X position in world frame",
            default_value=TextSubstitution(text="-4.0"),
        ),
        DeclareLaunchArgument(
            "spawn_y",
            description="Robot spawn Y position in world frame",
            default_value=TextSubstitution(text="-3.0"),
        ),
        DeclareLaunchArgument(
            "spawn_z",
            description="Robot spawn Z position in world frame",
            default_value=TextSubstitution(text="0.43"),
        ),
        DeclareLaunchArgument(
            "spawn_yaw",
            description="Robot spawn yaw (rad) in world frame",
            default_value=TextSubstitution(text="0.0"),
        ),
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_node,
        # robot_joint_controller_node,  # Spawn in rl_sim.cpp
        joy_node,
        param_node,
        display_launch,
    ])

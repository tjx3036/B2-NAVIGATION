import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('rl_sar'),
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_dwa_planner',
            default_value='true',
            description='Use custom DWA as local planner instead of Nav2 RegulatedPurePursuit'),

        # Pure DWA mode: keep controller_server for local_costmap/recovery services.
        # Its FollowPath action is fully remapped to /nav2_follow_path to avoid conflict.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': True},
                {'node_names': [
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator'
                ]}
            ],
            condition=IfCondition(LaunchConfiguration('use_dwa_planner'))
        ),
        # Nav2 controller mode (fallback): keep controller_server in lifecycle managed list
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': True},
                {'node_names': [
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator'
                ]}
            ],
            condition=UnlessCondition(LaunchConfiguration('use_dwa_planner'))
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        # Controller in normal Nav2 mode (without custom DWA)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/cmd_vel', '/nav2_cmd_vel'),
                ('cmd_vel', 'nav2_cmd_vel'),
                # Remap all FollowPath action endpoints to avoid /follow_path conflict
                ('/follow_path/_action/send_goal', '/nav2_follow_path/_action/send_goal'),
                ('/follow_path/_action/get_result', '/nav2_follow_path/_action/get_result'),
                ('/follow_path/_action/cancel_goal', '/nav2_follow_path/_action/cancel_goal'),
                ('/follow_path/_action/feedback', '/nav2_follow_path/_action/feedback'),
                ('/follow_path/_action/status', '/nav2_follow_path/_action/status'),
                ('follow_path/_action/send_goal', 'nav2_follow_path/_action/send_goal'),
                ('follow_path/_action/get_result', 'nav2_follow_path/_action/get_result'),
                ('follow_path/_action/cancel_goal', 'nav2_follow_path/_action/cancel_goal'),
                ('follow_path/_action/feedback', 'nav2_follow_path/_action/feedback'),
                ('follow_path/_action/status', 'nav2_follow_path/_action/status'),
            ],
            condition=IfCondition(LaunchConfiguration('use_dwa_planner'))
        ),
        # Controller in normal Nav2 mode (without custom DWA)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/cmd_vel', '/nav2_cmd_vel')],
            condition=UnlessCondition(LaunchConfiguration('use_dwa_planner'))
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/cmd_vel', '/nav2_cmd_vel')]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        # 自定义 DWA 局部规划器：订阅 /plan，发布 /cmd_vel，替代 Nav2 controller 的输出
        # controller_server 仍运行（为 BT 提供 FollowPath 接口，并发布 local_costmap），但其 /nav2_cmd_vel 不被使用
        Node(
            package='dwa_local_planner',
            executable='sac_dwa_node',
            name='dwa_local_planner',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_dwa_planner')),
        ),
    ])
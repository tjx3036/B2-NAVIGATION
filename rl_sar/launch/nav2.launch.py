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
    params_filtered = os.path.join(
        get_package_share_directory('rl_sar'),
        'config',
        'nav2_params_scan_filtered.yaml'
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
        DeclareLaunchArgument(
            'use_scan_bridge',
            default_value='true',
            description='Enable velodyne PointCloud2 to LaserScan bridge'),
        DeclareLaunchArgument(
            'use_dynamic_scan_filter',
            default_value='true',
            description='Filter dynamic obstacles from scan before costmap (reduces ghost inflation)'),
        DeclareLaunchArgument(
            'alias_map_to_odom',
            default_value='false',
            description='Treat map and odom as identical in custom DWA node'),

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
            parameters=[params_file, params_filtered, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        # Controller in DWA mode: keep Nav2 controller for costmaps & recovery.
        # NOTE: Do NOT remap FollowPath action here, otherwise bt_navigator
        #       cannot find the 'follow_path' action server and bringup fails.
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, params_filtered, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/cmd_vel', '/nav2_cmd_vel'),
                ('cmd_vel', 'nav2_cmd_vel'),
            ],
            condition=IfCondition(LaunchConfiguration('use_dwa_planner'))
        ),
        # Controller in normal Nav2 mode (without custom DWA)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                params_file,
                params_filtered,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[('/cmd_vel', '/nav2_cmd_vel')],
            condition=UnlessCondition(LaunchConfiguration('use_dwa_planner'))
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, params_filtered, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/cmd_vel', '/nav2_cmd_vel')]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, params_filtered, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='velodyne_laserscan',
            executable='velodyne_laserscan_node',
            name='velodyne_laserscan_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('velodyne_points', '/velodyne_points'),
                ('scan', '/scan'),
            ],
            condition=IfCondition(LaunchConfiguration('use_scan_bridge')),
        ),
        # 动态障碍物过滤：/scan -> /scan_filtered，减少 costmap 中 ghost 膨胀带
        Node(
            package='dwa_local_planner',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'enabled': LaunchConfiguration('use_dynamic_scan_filter')},
            ],
            condition=IfCondition(LaunchConfiguration('use_scan_bridge')),
        ),
        # 自定义 DWA 局部规划器：订阅 /plan，发布 /cmd_vel，替代 Nav2 controller 的输出
        # controller_server 仍运行（为 BT 提供 FollowPath 接口，并发布 local_costmap），但其 /nav2_cmd_vel 不被使用
        Node(
            package='dwa_local_planner',
            executable='sac_dwa_node',
            name='dwa_local_planner',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'alias_map_to_odom': LaunchConfiguration('alias_map_to_odom')},
            ],
            condition=IfCondition(LaunchConfiguration('use_dwa_planner')),
        ),
        # 非 DWA 模式：中继 /scan -> /scan_for_costmap，使 costmap 正常工作
        Node(
            package='dwa_local_planner',
            executable='scan_relay_node',
            name='scan_relay',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=UnlessCondition(LaunchConfiguration('use_dwa_planner')),
        ),
    ])
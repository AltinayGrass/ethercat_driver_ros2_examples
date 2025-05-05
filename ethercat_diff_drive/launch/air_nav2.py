from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    nav2_params = PathJoinSubstitution([
        FindPackageShare('ethercat_diff_drive'),
        'config',
        'nav2_params.yaml'
    ])

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    nav2_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped' )],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped' )],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),
        
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            remappings=remappings + [('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped' )],
            ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': [
                            'controller_server',
                            'planner_server',
                            'bt_navigator',
                            'behavior_server',
                            'docking_server',
                            # 'local_costmap',
                            # 'global_costmap'
                        ]}]
        )
    ]

    return LaunchDescription([
        declare_use_sim_time_arg
    ] + nav2_nodes)

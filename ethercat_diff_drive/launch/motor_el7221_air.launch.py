# Copyright 2023 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(package='ethercat_diff_drive').find('ethercat_diff_drive')
    use_sim_time = LaunchConfiguration('use_sim_time')


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='air_urdf_el7221.xacro',
            description='URDF/XACRO description file with the axis.',
        )
    )

    declared_arguments.append(declare_use_sim_time_cmd)

    description_file = LaunchConfiguration('description_file')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ethercat_diff_drive"),
                    "description/config",
                    description_file,
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "controllers_air.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{'use_sim_time': use_sim_time},robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
    )

    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "-c", "/controller_manager"],
    )

    delay_gpio_after_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[gpio_controller_spawner],
        )
    )
    # velocity_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["velocity_controller", "-c", "/controller_manager"],
    # )

    # effort_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["effort_controller", "-c", "/controller_manager"],
    # )

    nodes = [
        #start_gazebo_server_cmd,
        # start_gazebo_client_cmd,
        #spawn_entity,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        delay_gpio_after_diff_drive_controller_spawner,
        # velocity_controller_spawner,
        # effort_controller_spawner,
    ]

    return LaunchDescription(
        declared_arguments +
        nodes)

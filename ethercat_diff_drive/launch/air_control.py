import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node



def generate_launch_description():

    pkg_ethercat_diff_drive_dir = get_package_share_directory('ethercat_diff_drive') # Bunu kullanmak daha iyi

    # --- Launch Argüman Tanımlamaları ---
    use_sim_time = LaunchConfiguration('use_sim_time') # Varsayılan değer Declare'de
    description_file = LaunchConfiguration('description_file')

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false', # String 'false'
            description='Use simulation (Gazebo) clock if true')
    )
 
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='air_urdf_el7221.xacro',
            description='URDF/XACRO description file with the axis.',
        )
    )


    # --- URDF ve ros2_control ---
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([pkg_ethercat_diff_drive_dir, "description/config", description_file]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([pkg_ethercat_diff_drive_dir, "config", "controllers_air.yaml"])

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
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "--controller-manager", "/controller_manager"],
    )

    delay_gpio_after_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[gpio_controller_spawner],
        )
    )

    light_control_node = Node( # İsim verildi
        package='light_control',
        executable='light_control',
        name='light_control',
        output='screen'
    )

    bms_status_node = Node(
        package='air_bms',                   # BMS düğümünün paketi
        executable='bms_status_node_gmn',    # Çalıştırılacak dosya adı
        name='bms_status_node',              # Düğüme verilecek isim (isteğe bağlı ama önerilir)
        output='screen',                     # Çıktıyı ekrana ver
        parameters=[{                        # Parametreleri liste içinde sözlük olarak ver
            'update_interval_seconds': 5.0
        }]
    )
    # --- Başlatılacak Düğümlerin Listesi ---
    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        delay_gpio_after_diff_drive_controller_spawner,
        light_control_node,
        bms_status_node, 
    ]

    return LaunchDescription(declared_arguments + nodes)
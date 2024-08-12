import launch
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='ethercat_diff_drive').find('ethercat_diff_drive')
    default_model_path = os.path.join(pkg_share, 'description/config/motor_drive.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ethercat_diff_drive"),
            "config",
            "controllers.yaml",
        ]
    )

    twist_timestamp_remover_config= PathJoinSubstitution(
        [
            FindPackageShare("twist_timestamp_remover"),
            "config",
            'params.yaml'
        ]
    )

    twist_timestamp_remover_node= launch_ros.actions.Node(
        package='twist_timestamp_remover',
        executable='twist_timestamp_remover',
        name='twist_timestamp_remover',
        parameters=[twist_timestamp_remover_config]
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output="both",
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])},{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )

    joint_state_publisher_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    diff_drive_controller_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'motor_drive', '-topic', 'robot_description'],
        output='screen'
    )
    robot_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, robot_controllers],
        output="both",
    )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        twist_timestamp_remover_node,
        robot_control_node,
        diff_drive_controller_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        #rviz_node
    ])
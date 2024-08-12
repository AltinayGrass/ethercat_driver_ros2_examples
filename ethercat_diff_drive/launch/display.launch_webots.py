import os
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

def generate_launch_description():
    pkg_share = FindPackageShare(package='ethercat_diff_drive').find('ethercat_diff_drive')
    default_model_path = os.path.join(pkg_share, 'description/config/motor_drive_webots_urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    
    use_rviz = LaunchConfiguration('rviz', default=False)
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam_toolbox = LaunchConfiguration('slam_toolbox', default=False)
    use_slam_cartographer = LaunchConfiguration('slam_cartographer', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=os.path.join(pkg_share, 'world', 'my_world.wbt'),
        ros2_supervisor=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', default_model_path]),
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diff_drive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    mappings = [('/diff_drive_controller/cmd_vel', '/cmd_vel'), ('/diff_drive_controller/odom', '/odom')]
    ros2_control_params = os.path.join(pkg_share, 'config', 'controllers_webots.yaml')
    robot_description_path = os.path.join(pkg_share, 'description/config', 'motor_drive_webots.urdf')
    
    robot_driver_igh = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'robot_description': Command(['xacro ', default_model_path])}, 
                    ros2_control_params],
        output="both",
        )
    
    robot_driver = WebotsController(
            robot_name='motor_drive',
            parameters=[
                {'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': False},
                ros2_control_params
            ],
            remappings=mappings,
            # respawn=True
        )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = launch.actions.DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = launch.actions.DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        )
    
    # Navigation
    navigation_nodes = []
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    cartographer_config_basename = 'cartographer.lua'
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', cartographer_config_basename],
        condition=launch.conditions.IfCondition(use_slam_cartographer))
    navigation_nodes.append(cartographer)

    grid_executable = 'cartographer_occupancy_grid_node'
    cartographer_grid = Node(
        package='cartographer_ros',
        executable=grid_executable,
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05'],
        condition=launch.conditions.IfCondition(use_slam_cartographer))
    navigation_nodes.append(cartographer_grid)

    toolbox_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    slam_toolbox = Node(
        parameters=[toolbox_params,
                    {'use_sim_time':  use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=launch.conditions.IfCondition(use_slam_toolbox)
    )
    navigation_nodes.append(slam_toolbox)    

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=robot_driver,
        nodes_to_start= navigation_nodes + ros_control_spawners
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        declare_x_position_cmd,
        declare_y_position_cmd,
        webots,
        webots._supervisor,
        robot_state_publisher,
        #footprint_publisher,
        robot_driver,
        #robot_driver_igh,
        waiting_nodes,
        #rviz_node
    ])
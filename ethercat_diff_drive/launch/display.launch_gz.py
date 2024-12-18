import launch
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='ethercat_diff_drive').find('ethercat_diff_drive')
    default_model_path = os.path.join(pkg_share, 'description/config/motor_drive_webots_urdf_gz.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    use_slam_toolbox = LaunchConfiguration('slam_toolbox', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    world = PathJoinSubstitution(
        [
            FindPackageShare("ethercat_diff_drive"),
            "world",
            "my_world.sdf",
        ]
    )

    sdf_file = os.path.join(pkg_share,'world', 'motor_drive' , 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    gzserver_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ethercat_diff_drive"),
            "config",
            "controllers_webots.yaml",
        ]
        )

    bridge_params = os.path.join(
        get_package_share_directory('ethercat_diff_drive'),
        'config',
        'gz_bridge.yaml'
        )
    
    start_gazebo_ros_bridge_cmd = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
        )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output="both",
        parameters=[
            # {'robot_description': robot_desc},
            {'robot_description': Command(['xacro ', default_model_path])},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
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

    spawn_entity = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'motor_drive',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.015'
        ],
        output='screen',
        )

    robot_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'robot_description': Command(['xacro ', default_model_path])}, 
                    robot_controllers],
        output="both",
        )
    
    # Navigation
    toolbox_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    slam_toolbox = launch_ros.actions.Node(
        parameters=[toolbox_params,
                    {'use_sim_time':  use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=launch.conditions.IfCondition(use_slam_toolbox)
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
        declare_x_position_cmd,
        declare_y_position_cmd,
        gzserver_cmd,        
        gzclient_cmd,
        start_gazebo_ros_bridge_cmd,
        robot_control_node,
        diff_drive_controller_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        slam_toolbox,
        #spawn_entity,
        #rviz_node
    ])
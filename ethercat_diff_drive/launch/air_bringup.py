import os

from ament_index_python.packages import get_package_share_directory
# SetRemap artık doğrudan kullanılmıyor, kaldırılabilir
# from launch_ros.actions import SetRemap

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution, PythonExpression # PythonExpression eklendi
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
# IfCondition ve UnlessCondition zaten vardı, PythonExpression için de gerekli olabilir
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(package='ethercat_diff_drive').find('ethercat_diff_drive')
    pkg_ethercat_diff_drive_dir = get_package_share_directory('ethercat_diff_drive') # Bunu kullanmak daha iyi
    pkg_realsense2_camera_dir = get_package_share_directory('realsense2_camera')
    pkg_apriltag_draw_dir = get_package_share_directory('apriltag_draw')

    # --- Launch Argüman Tanımlamaları ---
    use_sim_time = LaunchConfiguration('use_sim_time') # Varsayılan değer Declare'de
    camera_frame_type = LaunchConfiguration('camera_frame_type')
    camera_ns = LaunchConfiguration('camera_ns')
    tag_family = LaunchConfiguration('tag_family')
    tag_id = LaunchConfiguration('tag_id')
    delete_db = LaunchConfiguration('delete_db_on_start')
    description_file = LaunchConfiguration('description_file')
    camera_off = LaunchConfiguration('camera_off') # Yeni argümanı tanımla

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false', # String 'false'
            description='Use simulation (Gazebo) clock if true')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='false',
            description='If true, deletes the RTAB-Map database on startup.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='air_urdf_el7221.xacro',
            description='URDF/XACRO description file with the axis.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='camera_frame_type',
            default_value='_optical_frame',
            description='Type of camera frame to use (e.g., _depth_optical_frame, _optical_frame)'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='camera_ns',
            default_value='camera',
            description='Namespace for the camera and AprilTag nodes'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='tag_family',
            default_value='tag36h11',
            description='Family of AprilTag being used'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='tag_id',
            default_value='7',
            description='ID of the AprilTag being used'
        )
    )
    # --- YENİ: camera_off argümanı ---
    declared_arguments.append(
        DeclareLaunchArgument(
            name='camera_off',
            default_value='false', # Varsayılan olarak kamera AÇIK
            description='If true, disables camera-related nodes (Realsense, RTABMap, AprilTag, etc.).'
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

    # --- RTAB-Map ---
    rtabmap_params = PathJoinSubstitution([
        pkg_ethercat_diff_drive_dir, 'config', 'rtabmap_params.yaml'
    ])

    # RTAB-Map düğümlerini koşullu olarak tanımla
    # Koşul 1: delete_db == true VE camera_off == false
    rtabmap_node_d = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params,
            {'use_sim_time': use_sim_time}, # Eklendi
            {'Mem/IncrementalMemory': 'true'}, # SLAM Modu
            # {'MinObstacleHeight': '0.06'}, # Bunlar Nav2 Costmap içindir
            # {'MaxObstacleHeight': '1.06'},
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('odom', '/diff_drive_controller/odom'),
            ('imu', '/camera/imu') # IMU remapping
        ],
        arguments=['-d'], # Veritabanını sil
        # --- Koşul: delete_db true VE camera_off false ise çalıştır ---
        condition=IfCondition(PythonExpression(
            ["'", delete_db, "' == 'true' and '", camera_off, "' == 'false'"]
        ))
    )

    # Koşul 2: delete_db == false VE camera_off == false
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap', # İki düğüm aynı anda çalışmayacağı için isim aynı olabilir
        output='screen',
        parameters=[
            rtabmap_params,
            {'use_sim_time': use_sim_time}, # Eklendi
            {'Mem/IncrementalMemory': 'false'}, # Localization Modu (veya YAML'da ayarla)
            # {'MinObstacleHeight': '0.06'}, # Nav2
            # {'MaxObstacleHeight': '1.06'}, # Nav2
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('odom', '/diff_drive_controller/odom'),
            ('imu', '/camera/imu') # IMU remapping
        ],
        # --- Koşul: delete_db false VE camera_off false ise çalıştır ---
        condition=IfCondition(PythonExpression(
            ["'", delete_db, "' == 'false' and '", camera_off, "' == 'false'"]
        ))
    )

    # --- Realsense Kamera ---
    # realsense_launch_backup kaldırıldı, kullanılmıyor.
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_realsense2_camera_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            # QoS için config_file argümanı burada eklenebilir (önceki cevaplardaki gibi)
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_sync': 'true',
            'align_depth.enable':'true',
            'enable_gyro': 'true', # IMU önerilir
            'enable_accel': 'true',# IMU önerilir
            'unite_imu_method': 'linear_interpolation',
            'camera_name': 'camera',
            'camera_namespace': camera_ns, # Argümanı kullan
            'publish_tf': 'true',
            'tf_publish_rate': '30.0',
            'pointcloud.enable': 'false',
            'use_sim_time': use_sim_time,
            # 'depth_module.profile':'640x480x30',
            # 'rgb_camera.profile':'640x480x30',
        }.items(),
        # --- Koşul: camera_off false ise çalıştır ---
        condition=UnlessCondition(camera_off)
    )

    # --- Depth to LaserScan ---
    depth_to_scan_params = {
            'scan_height': 10, # Değeri deneyerek ayarlayın
            'range_min': 0.3,
            'range_max': 4.0,
            'output_frame': 'camera_link', # TF ağacınızla eşleşmeli!
            'scan_time': 0.033,
            'use_sim_time': use_sim_time
        }

    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[depth_to_scan_params],
        remappings=[
            ('depth', [camera_ns, '/aligned_depth_to_color/image_raw']), # Namespace kullanıldı
            ('depth_camera_info', [camera_ns, '/aligned_depth_to_color/camera_info']), # Namespace kullanıldı
            ('scan', '/scan')
        ],
        # --- Koşul: camera_off false ise çalıştır ---
        condition=UnlessCondition(camera_off)
    )

    # --- Apriltag ---
    # static_tf_virtual_scan kaldırıldı, kullanılmıyor gibi.
    apriltag_ros_params = PathJoinSubstitution([
        pkg_ethercat_diff_drive_dir, "config", "tags_36h11.yaml", # tag_family argümanını burada kullanabilirsiniz
    ])

    apriltag_ros_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace=camera_ns, # Node'u da namespace içine al
        parameters=[
            apriltag_ros_params,
            {'use_sim_time': use_sim_time}
            ],
        remappings=[
            # Namespace içinde olduğu için göreli isimler kullanılabilir
            ('image_rect', 'color/image_raw') ,
            ('camera_info', 'color/camera_info'),
            ('detections', '/detections') # Global topic name for detections
        ],
        # --- Koşul: camera_off false ise çalıştır ---
        condition=UnlessCondition(camera_off)
    )

    # --- Apriltag Draw ---
    apriltag_draw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_apriltag_draw_dir, 'launch', 'draw.launch.py')
        ),
        launch_arguments={
            # draw.launch.py'nin beklediği argümanları doğru şekilde iletin
            'image_topic': [camera_ns, '/color/image_raw'],
            'detections_topic': '/detections'
        }.items(),
        # --- Koşul: camera_off false ise çalıştır ---
        condition=UnlessCondition(camera_off)
    )

    # --- Detected Dock Pose Publisher ---
    # TF frame isimlerini birleştirmek için PathJoinSubstitution veya PythonExpression kullanmak daha iyi
    parent_frame_sub = PathJoinSubstitution([camera_ns, '_color', camera_frame_type])
    child_frame_sub = PathJoinSubstitution([tag_family, ':', tag_id])

    start_detected_dock_pose_publisher = Node(
        package='light_control', # Paketin adı doğru mu?
        executable='detected_dock_pose_publisher',
        name='detected_dock_pose_publisher',
        parameters=[{
            # 'parent_frame': [camera_ns, TextSubstitution(text='_color'), camera_frame_type], # Bu liste birleştirilmez
            # 'child_frame': [tag_family, TextSubstitution(text=':'), tag_id], # Bu liste birleştirilmez
            'parent_frame': parent_frame_sub, # Birleştirilmiş substituion
            'child_frame': child_frame_sub,   # Birleştirilmiş substituion
            'publish_rate': 10.0,
            'use_sim_time': use_sim_time
        }],
        output='screen',
        # --- Koşul: camera_off false ise çalıştır ---
        condition=UnlessCondition(camera_off)
    )

    # --- Başlatılacak Düğümlerin Listesi ---
    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        delay_gpio_after_diff_drive_controller_spawner,
        light_control_node, # İsim verildi
        # --- Koşullu Düğümler/Launch'lar ---
        realsense_launch,
        depthimage_to_laserscan_node,
        rtabmap_node, # Koşulu içinde tanımlı
        rtabmap_node_d, # Koşulu içinde tanımlı
        apriltag_ros_node,
        apriltag_draw_launch,
        start_detected_dock_pose_publisher,
    ]

    return LaunchDescription(declared_arguments + nodes)
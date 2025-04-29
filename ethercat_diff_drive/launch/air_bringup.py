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

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetRemap

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(package='ethercat_diff_drive').find('ethercat_diff_drive')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    camera_frame_type = LaunchConfiguration('camera_frame_type')
    camera_ns = LaunchConfiguration('camera_ns')
    tag_family = LaunchConfiguration('tag_family')
    tag_id = LaunchConfiguration('tag_id')

    declare_camera_frame_type_cmd = DeclareLaunchArgument(
        name='camera_frame_type',
        default_value='_optical_frame',
        description='Type of camera frame to use (e.g., _depth_optical_frame, _optical_frame)'
    )

    declare_camera_ns_cmd = DeclareLaunchArgument(
        name='camera_ns',
        default_value='camera',
        description='Namespace for the camera and AprilTag nodes'
    )

    declare_tag_family_cmd = DeclareLaunchArgument(
        name='tag_family',
        default_value='tag36h11',
        description='Family of AprilTag being used'
    )

    declare_tag_id_cmd = DeclareLaunchArgument(
        name='tag_id',
        default_value='7',
        description='ID of the AprilTag being used'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    delete_db_arg = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='false',  # Varsayılan olarak veritabanını SİLME
        description='If true, deletes the RTAB-Map database on startup via the -d flag.'
    )


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
    declared_arguments.append(delete_db_arg)
    declared_arguments.append(declare_camera_frame_type_cmd)
    declared_arguments.append(declare_camera_ns_cmd)
    declared_arguments.append(declare_tag_family_cmd)
    declared_arguments.append(declare_tag_id_cmd)

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
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
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

    light_control=Node(
        package='light_control',
        executable='light_control',
        output='screen'
    )

    # RTAB-Map Node
    rtabmap_params = PathJoinSubstitution([
        pkg_share,
        'config',
        'rtabmap_params.yaml'  # Bu dosya birazdan vereceğim
    ])
    delete_db = LaunchConfiguration('delete_db_on_start')
    
    rtabmap_node_d = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params,
            {'Mem/IncrementalMemory': 'true'}, # Haritayı güncelle (SLAM Modu)
            {'MinObstacleHeight': '0.06'},
            {'MaxObstacleHeight': '1.06'},
            ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'), #'/camera_info'),#
            # ('rgbd_image', '/camera/rgbd'),
            ('odom', '/diff_drive_controller/odom')
        ],
        arguments=['-d'],
        condition=IfCondition(delete_db)
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params,
            {'Mem/IncrementalMemory': 'false'}, # Haritayı güncelleme
            #{'Mem/IncrementalMemory': 'true'}, # Haritayı güncelle (SLAM Modu)
            {'MinObstacleHeight': '0.06'},
            {'MaxObstacleHeight': '1.06'},            
            ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'), # '/camera_info'),
            # ('rgbd_image', '/camera/rgbd'),
            ('odom', '/diff_drive_controller/odom')
        ],
        condition=UnlessCondition(delete_db)
    )

    realsense_launch_backup = GroupAction(
        actions=[
            SetRemap(src='/camera/color/camera_info',dst='camera_info'),
            IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                            get_package_share_directory('realsense2_camera'),
                            'launch'),
                            '/rs_launch.py']),
                    launch_arguments={
                        'enable_rgbd': 'true',
                        'enable_sync': 'true',
                        'align_depth.enable':'true',
                        'camera_name': 'camera',
                        'camera_namespace': '',
                        # 'depth_module.depth_profile':'640x480x30',
                        # 'rgb_camera.color_profile':'640x480x30',
                        # 'pointcloud.enable':'true'
                        }.items(),
                    )
        ]
    
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch'),
                '/rs_launch.py']),
        launch_arguments={
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable':'true',
            'camera_name': 'camera',
            'camera_namespace': '',
            # 'depth_module.depth_profile':'640x480x30',
            # 'rgb_camera.color_profile':'640x480x30',
            # 'pointcloud.enable':'true'
            }.items(),
        )

    depth_to_scan_params = {
            'scan_height': 2,           # Görüntünün ortasından kullanılacak piksel satırı sayısı (1 ile başlayıp deneyebilirsiniz)
            'range_min': 0.3,           # Minimum geçerli mesafe (metre) - Realsense D435i için uygun
            'range_max': 4.0,           # Maksimum geçerli mesafe (metre) - Ortamınıza göre ayarlayın
            'output_frame': 'camera_link', # Lazer taramasının yayınlanacağı TF frame'i - Çok Önemli!
                                        # TF ağacınıza göre 'camera_link' veya başka bir frame olabilir.
            'scan_time': 0.033,         # Tarama süresi (1/frekans), yaklaşık ~30Hz için
            # 'inf_epsilon': 1.0        # Sonsuz değerler için epsilon (varsayılanı genellikle iyidir)
            'use_sim_time': use_sim_time # Simülasyon zamanını kullan
        }
    
    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[depth_to_scan_params], # Parametreleri buradan veriyoruz
        remappings=[
            # Abone olunacak konular: Hizalanmış derinlik ve ilgili kamera bilgisi
            ('depth', '/camera/aligned_depth_to_color/image_raw'),
            ('depth_camera_info', '/camera/aligned_depth_to_color/camera_info'),
            # Yayınlanacak konu: Standart lazer tarama konusu
            ('scan', '/scan')
        ]
    )
    static_tf_virtual_scan = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_virtual_scan',
            arguments=[
                '0.3', '0.0', '0.75', # X, Y, Z offset (metre) base_link'e göre
                '0', '0', '0',       # Roll, Pitch, Yaw (radyan) base_link'e göre
                'base_link',         # Parent frame
                'virtual_scan_frame' # Child frame (Yeni sanal çerçevemiz)
            ],
            output='screen'
        )
    apriltag_ros_params = PathJoinSubstitution(
        [
            pkg_share,
            "config",
            "tags_36h11.yaml",
        ]
        )
    
    apriltag_ros_node = Node(
            package='apriltag_ros',
            executable='apriltag_node',
            parameters=[apriltag_ros_params],
            remappings=[('image_rect', '/camera/color/image_raw') ,
                        ('camera_info', '/camera/color/camera_info'),
                        ],
        )
    # apriltag_draw_node = Node(
    #         package='apriltag_draw',
    #         executable='apriltag_draw_node',
    #         # prefix=['xterm -e gdb -ex run --args'],
    #         namespace='camera/color',
    #         # parameters=[{'image_transport': trans, 'max_queue_size': 200}],
    #         remappings=[('tags', '/detections'),
    #                     ('image_raw', '/camera/color/image_raw'),
    #                     ],
    #     )
    
    pkg_apriltag_draw = get_package_share_directory('apriltag_draw')

    apriltag_draw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_apriltag_draw, 'launch', 'draw.launch.py')
        ),
        # Pass the required launch arguments to draw.launch.py
        launch_arguments={
            # 'camera' argument for draw.launch.py mapped to the Realsense color image topic
            'camera': '/camera/color', # Should match input to detector ideally
            # 'tags' argument for draw.launch.py mapped to the detector's output topic
            'tags': '/detections' # Default output topic of apriltag_node
        }.items(),
    )
    
    start_detected_dock_pose_publisher = Node(
        package='light_control',
        executable='detected_dock_pose_publisher',
        parameters=[{
            'parent_frame': [camera_ns, TextSubstitution(text='_color'), camera_frame_type],
            'child_frame': [tag_family, TextSubstitution(text=':'), tag_id],
            'publish_rate': 10.0
        }],
        output='screen'
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        delay_gpio_after_diff_drive_controller_spawner,
        #static_tf_virtual_scan,
        light_control,
        realsense_launch,
        depthimage_to_laserscan_node,
        rtabmap_node,
        rtabmap_node_d,
        apriltag_ros_node,
        apriltag_draw_launch,
        start_detected_dock_pose_publisher,
    ]

    return LaunchDescription(
        declared_arguments +
        nodes)
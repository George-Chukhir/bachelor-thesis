import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    b2_fusion_dir = get_package_share_directory('b2_thesis_fusion')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    bag_path = LaunchConfiguration(
        'bag_path', 
        default='/home/stringer/b2_ws/src/raw_bag/hangar/wifi_fourth_bag' 
    )
    
    rviz_config_file = os.path.join(b2_fusion_dir, 'rviz', 'config_l4.rviz')

    ekf_config_l3 = os.path.join(b2_fusion_dir, 'config', 'ekf_l3.yaml')

    output_tum_file = '/home/stringer/b2_ws/src/b2_thesis_fusion/separate_trajectories/bag4_trajectories/l4_traj.tum'

    yaml_filename = '/home/stringer/b2_ws/src/b2_thesis_fusion/maps/2d_maps/map_bag_4/map_bag_4.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('bag_path', default_value=bag_path),

        DeclareLaunchArgument('output_tum_file', 
                              default_value=output_tum_file, 
                              description='Path to the output TUM file for the L4 trajectory'),

        DeclareLaunchArgument('yaml_filename', 
                              default_value=yaml_filename, 
                              description='Path to the YAML file for the 2D map'),

        # 1. ROSBag (Legged odometry, IMU, LiDAR)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock', '--remap', '/tf_static:=/tf_static_old', '/tf:=/tf_old'],
            output='screen'
        ),

        ###
        # Static TFs
        ###
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['--x', '0', '--y', '-0.02341', '--z', '0.04927', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                      '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='correct_velodyne_tf',
            arguments=['0.3', '0.0', '0.4', '0.0', '0.0', '0.0', 'base_link', 'velodyne'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        ###
        # KISS-ICP Odometry
        ###
        Node(
            package='kiss_icp',
            executable='kiss_icp_node',
            name='kiss_icp_node',
            remappings=[
                ('pointcloud_topic', '/velodyne_points'),
                ('odometry', '/kiss/odometry'),
            ],
            parameters=[{
                'base_frame': 'base_link',
                'lidar_odom_frame': 'odom',
                'publish_odom_tf': False,
                'data.max_range': 20.0,
                'data.min_range': 0.5,
                'data.deskew': True,
                'position_covariance': 0.005,
                'orientation_covariance': 0.005,
            }],
        ),

        ###
        # EKF L3 (local odometry fusion)
        ###
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_l3',
            parameters=[ekf_config_l3, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered_l4')]
        ),

        Node(
            package='b2_thesis_fusion',
            executable='path_tracker',
            name='tracker_l4',
            parameters=[{
                'odom_topic': '/odometry/filtered_l4', 
                'path_topic': '/trajectory/l4', 
                'frame_id': 'map', 
                'robot_frame': 'base_link',
                'use_sim_time': use_sim_time,
                'output_tum_file': output_tum_file 
            }],   
        ),

        ###
        # AMCL (Global localization on the map)
        ###
        # 1. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'yaml_filename': yaml_filename,
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[('cloud_in', '/velodyne_points'), ('scan', '/scan')],
            parameters=[{
                'target_frame': 'velodyne',
                'min_height': 0.2,
                'max_height': 1.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087, # ~0.5 degree
                'scan_time': 0.1,
                'range_min': 0.5,
                'range_max': 20.0,
                'use_sim_time': use_sim_time
            }]
        ),

        # 3. AMCL Node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'scan_topic': '/scan',
                'set_initial_pose': True, 
                'initial_pose': [0.0, 0.0, 0.0], 
            }]
        ),

        # 4. Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),

        ###
        # RViz
        ###
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
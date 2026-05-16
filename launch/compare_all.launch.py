import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    b2_fusion_dir = get_package_share_directory('b2_thesis_fusion')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    bag_path = LaunchConfiguration(
        'bag_path', 
        default='/home/stringer/b2_ws/src/raw_bag/test_record_raw' 
    )
    
    rviz_config_file = os.path.join(b2_fusion_dir, 'rviz', 'fusion_config.rviz') 

    # Configs for different EKF layers
    ekf_config_l2 = os.path.join(b2_fusion_dir, 'config', 'ekf_l2.yaml')
    ekf_config_l3 = os.path.join(b2_fusion_dir, 'config', 'ekf_l3.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('bag_path', default_value=bag_path),

        # 1. ROSBag (Legged odometry, IMU, LiDAR и MO-CAP)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock', '--remap', '/tf_static:=/tf_static_old'],
            output='screen'

        ),

        ###
        #Static TFs
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
        # l1 - Legged Odometry (Raw)
        ###

        # 2. Retranslator for creating a constant path (nav_msgs/Path) from Level 1 Odometry (Raw Legged Odometry)
        Node(
            package='b2_thesis_fusion',
            executable='path_tracker',
            name='tracker_l1',
            parameters=[{'odom_topic': '/robot/base/odom', 'path_topic': '/trajectory/l1', 'frame_id': 'odom', 'use_sim_time': use_sim_time}],
        ),


        ### 
        # l2 - Legged Odometry + IMU (EKF)
        ###
        
        # 3. Node EKF for layer 2 (Leg Odometry + IMU)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_l2',
            parameters=[ekf_config_l2, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered_l2')]
        ),
        
        # 4. Retranslator to create a path from L2 EKF output
        Node(
            package='b2_thesis_fusion',
            executable='path_tracker',
            name='tracker_l2',
            parameters=[{'odom_topic': '/odometry/filtered_l2', 'path_topic': '/trajectory/l2', 'frame_id': 'odom', 'use_sim_time': use_sim_time}],
        ),


        ###
        # l3 - Legged Odometry + IMU + KISS-ICP + EKF
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
                'data.deskew': False,
                'position_covariance': 0.005,
                'orientation_covariance': 0.005,
            }],
        ),



        # raw KISS-ICP
        Node(
            package='b2_thesis_fusion',
            executable='path_tracker',
            name='tracker_kiss_raw',
            parameters=[{
                'odom_topic': '/kiss/odometry', 
                'path_topic': '/trajectory/kiss_raw', 
                'frame_id': 'odom', 
                'use_sim_time': use_sim_time
            }],
        ),

        # b) EKF Node L3
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_l3',
            parameters=[ekf_config_l3, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered_l3')]
        ),

        # d) Retranslator to create a path from L3 EKF output
        Node(
            package='b2_thesis_fusion',
            executable='path_tracker',
            name='tracker_l3',
            parameters=[{'odom_topic': '/odometry/filtered_l3', 'path_topic': '/trajectory/l3', 'frame_id': 'odom', 'use_sim_time': use_sim_time}],
        ),

        # 6. TODO: L4 (AMCL) and Ground Truth MOCAP
        # We will add them here as we develop the package

        # RViz2 for visualizing all paths simultaneously
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])

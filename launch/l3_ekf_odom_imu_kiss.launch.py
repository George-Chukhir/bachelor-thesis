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
        default='/home/stringer/b2_ws/src/raw_bag/hangar/wifi_third_bag' 
    )
    
    ekf_config_l3 = os.path.join(b2_fusion_dir, 'config', 'ekf_l3.yaml')
    rviz_config_file = os.path.join(b2_fusion_dir, 'rviz', 'config_l3.rviz') 

    output_tum_file = '/home/stringer/b2_ws/src/b2_thesis_fusion/separate_trajectories/bag3_trajectories/l3_traj_new_kiss.tum'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true', 
            description='Use simulation (bag) time'
        ),
        DeclareLaunchArgument(
            'bag_path',
            default_value=bag_path,
            description='Path to the raw rosbag'
        ),
        DeclareLaunchArgument(
        'output_tum_file',
            default_value=output_tum_file,
            description='Path to the output TUM file for the L3 trajectory'
        ),


        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock', '--remap', '/tf_static:=/tf_static_old', '--remap', '/tf:=/tf_old'],
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
            arguments=['0.4', '0.0', '0.3', '0.0', '0.0', '0.0', 'base_link', 'velodyne'],
            parameters=[{'use_sim_time': use_sim_time}]
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
                'data.max_range': 40.0,
                'data.min_range': 0.8,
                'data.deskew': True,
                'data.voxel_size': 0.05,
                'max_points_per_voxel': 3,
                'position_covariance': 0.075,
                'orientation_covariance': 0.1,
            }],
        ),


        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_l3',
            output='screen',
            parameters=[ekf_config_l3],
            remappings=[
                ('odometry/filtered', '/odometry/filtered_l3')
            ]
        ),


        Node(
            package='b2_thesis_fusion',
            executable='path_tracker',
            name='tracker_l3',
            parameters=[{'odom_topic': '/odometry/filtered_l3', 
                         'path_topic': '/trajectory/l3', 
                         'frame_id': 'odom', 
                         'use_sim_time': use_sim_time, 
                          'output_tum_file': output_tum_file,
                           'marker_yaw_offset': 4.48}],
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])

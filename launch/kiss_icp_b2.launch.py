import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    b2_fusion_dir = get_package_share_directory('b2_thesis_fusion')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    bag_path = LaunchConfiguration(
        'bag_path', 
        default='/home/stringer/b2_ws/src/raw_bag/hangar/wifi_third_bag' 
    )

    output_tum_file = '/home/stringer/b2_ws/src/b2_thesis_fusion/separate_trajectories/bag3_trajectories/kiss_izolated_voxel_0_05_count_3.tum'


    rviz_config_file = os.path.join(b2_fusion_dir, 'rviz', 'kiss_izolated.rviz') 

    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('bag_path', default_value=bag_path),

         DeclareLaunchArgument(
        'output_tum_file',
            default_value=output_tum_file,
            description='Path to the output TUM file for the L3 trajectory'
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock', 
                 '--remap', '/tf_static:=/tf_static_old', '/tf:=/tf_old'],
            output='screen'

        ),
        Node(
            package='kiss_icp',
            executable='kiss_icp_node',
            name='kiss_icp_node',
            output='screen',
            remappings=[
                ('pointcloud_topic', '/velodyne_points'),
                ('odometry', '/kiss/odometry'),
            ],
            parameters=[{
                'base_frame': 'base_link',
                'lidar_odom_frame': 'odom',
                'publish_odom_tf': True,
                'data.max_range': 40.0,
                'data.min_range': 0.8,
                'data.deskew': True,
                'data.voxel_size': 0.05,
                'max_points_per_voxel': 3,
                'position_covariance': 0.1,
                'orientation_covariance': 0.1,
            }],
        ),
          Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['--x', '0', '--y', '-0.02341', '--z', '0.04927', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                      '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    
        # Static TF to correct the KISS-ICP (l3)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='correct_velodyne_tf',
            arguments=['0.4', '0.0', '0.3', '0.0', '0.0', '0.0', 'base_link', 'velodyne'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='b2_thesis_fusion',
            executable='path_tracker',
            name='tracker_l2',
            parameters=[{'odom_topic': '/kiss/odometry',
                         'path_topic': '/trajectory/kiss/isolated', 
                         'frame_id': 'odom', 
                         'use_sim_time': use_sim_time, 
                         'output_tum_file': output_tum_file,
                         'marker_yaw_offset': 4.48}],
        ),

    ])

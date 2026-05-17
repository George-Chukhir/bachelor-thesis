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
        default='/home/stringer/b2_ws/src/raw_bag/test_record_raw' 
    )
    
    ekf_config_l2 = os.path.join(b2_fusion_dir, 'config', 'ekf_l2.yaml')
    rviz_config_file = os.path.join(b2_fusion_dir, 'rviz', 'config_l2.rviz') 

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

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock',  '--delay', '3.0', '--remap', '/tf_static:=/tf_static_old', '--remap', '/tf:=/tf_old'],
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
        # l2 - Legged Odometry + IMU (EKF)
        ###
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_l2',
            output='screen',
            parameters=[ekf_config_l2],
            remappings=[
                ('odometry/filtered', '/odometry/filtered_l2')
            ]
        ),

        Node(
            package='b2_thesis_fusion',
            executable='path_tracker',
            name='tracker_l2',
            parameters=[{'odom_topic': '/odometry/filtered_l2', 
                         'path_topic': '/trajectory/l2', 
                         'frame_id': 'odom', 
                         'use_sim_time': use_sim_time, 
                         'output_tum_file': '/home/stringer/b2_ws/src/b2_thesis_fusion/separate_trajectories/l2_traj.tum'}],
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

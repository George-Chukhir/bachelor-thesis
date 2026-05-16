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
    rviz_config_file = os.path.join(b2_fusion_dir, 'rviz', 'vlp16_view.rviz') 

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

        # Нода проигрывания rosbag-файла 
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
            output='screen'
        ),

        # L2: Extended Kalman Filter (robot_localization)
        # /robot/base/odom + /imu/data ==> /odometry/filtered
        # TF: odom_filtered -> base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_l2',
            output='screen',
            parameters=[ekf_config_l2],
            remappings=[
                ('odometry/filtered', 'odometry/filtered_l2')
            ]
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

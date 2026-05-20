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
        # here u can change the default path to your rosbag file 
        default='/home/stringer/b2_ws/src/raw_bag/hangar/wifi_test_record_first' 
    )
    
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

        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
            output='screen'
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

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
     # https://github.com/ros-drivers/velodyne
     driver_pkg = FindPackageShare('velodyne_driver')
     pointcloud_pkg = FindPackageShare('velodyne_pointcloud')


     # 2. convert lowstate IMU messages to standard sensor_msgs/Imu format
     imu_bridge_node = Node(
         package='b2_thesis_fusion',
         executable='lowstate_imu_bridge',
         name='lowstate_imu_bridge',
         output='screen',
         parameters=[{
             'input_topic': '/lowstate',
             'output_topic': '/imu/data',
             'frame_id': 'imu_link',
             'quat_order': 'wxyz',
         }],
     )

     # 3. Launch Velodyne driver (publishes raw packets on /velodyne_packets)
     velodyne_driver_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             PathJoinSubstitution([driver_pkg, 'launch', 'velodyne_driver_node-VLP16-launch.py'])
         ),
         launch_arguments={
             'model': 'VLP16',
             'port': '2368',
         }.items()
     )

     # 4. convert Velodyne packets to PointCloud2 format (/velodyne_points)
     velodyne_transform_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             PathJoinSubstitution([pointcloud_pkg, 'launch', 'velodyne_transform_node-VLP16-launch.py'])
         ),
         launch_arguments={
             'model': 'VLP16',
         }.items()
     )

     return LaunchDescription([
         imu_bridge_node,
         velodyne_driver_launch,
         velodyne_transform_launch,
     ])

# ping <IP_OF_MACHINE_RUNNING_MOTIVE>
# Turn on streaming in Motive "Broadcast Frame Data"
# rerwite the IP addresses in the mocap4r2_optitrack_driver_params.yaml file

# terminal 1: ros2 launch kiss_icp_localisation record_raw_sensors.launch.py
# terminal 2: ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
# ros2 topic list
# ros2 topic hz /markers
# ros2 bag record -o my_recording_bag /tf_static /tf /robot/base/odom /imu/data /imu/data_sync /velodyne_points /markers /rigid_bodies /clock_sync/imu_sync_stats /clock_sync/vlp16_drift_ppm /clock_sync/vlp16_offset_ms 

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
     kiss_pkg = FindPackageShare('kiss_icp_localisation')
     driver_pkg = FindPackageShare('velodyne_driver')
     pointcloud_pkg = FindPackageShare('velodyne_pointcloud')
     mocap_pkg = FindPackageShare('mocap4r2_optitrack_driver')

         # robot_server_launch = IncludeLaunchDescription(
         #     PythonLaunchDescriptionSource(
         #         PathJoinSubstitution([kiss_pkg, 'launch', 'robot_server_odom.launch.py'])
         #     )
         # )

     imu_bridge_node = Node(
         package='kiss_icp_localisation',
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

     # https://github.com/ros-drivers/velodyne
     velodyne_driver_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             PathJoinSubstitution([driver_pkg, 'launch', 'velodyne_driver_node-VLP16-launch.py'])
         ),
         launch_arguments={
             'model': 'VLP16',
             'port': '2368',
         }.items()
     )

     velodyne_transform_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             PathJoinSubstitution([pointcloud_pkg, 'launch', 'velodyne_transform_node-VLP16-launch.py'])
         ),
         launch_arguments={
             'model': 'VLP16',
         }.items()
     )

     imu_static_tf = Node(
         package='tf2_ros',
         executable='static_transform_publisher',
         name='base_link_to_imu_link',
         arguments=['--x', '0', '--y', '-0.02341', '--z', '0.04927', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
         output='screen',
     )

     lidar_static_tf = Node(
         package='tf2_ros',
         executable='static_transform_publisher',
         name='base_link_to_velodyne',
         arguments=['0', '0', '0.3', '0', '0', '0', 'base_link', 'velodyne'],
         output='screen',
     )

     # 6. MOCAP OptiTrack Driver
     mocap_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             PathJoinSubstitution([mocap_pkg, 'launch', 'optitrack2.launch.py'])
         )
     )

     return LaunchDescription([
         imu_bridge_node,
         velodyne_driver_launch,
         velodyne_transform_launch,
         imu_static_tf,
         lidar_static_tf,
         mocap_launch
     ])


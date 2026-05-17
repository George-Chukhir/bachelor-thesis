"""Optional FASTLIO2 (+PGO loop closure) stack for Velodyne + IMU.

This launch is intended to be included from kiss_icp_localisation launches.
It keeps FASTLIO2 isolated under the /fastlio2 namespace and can optionally
start PGO loop closure for map-less operation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_lio_config = PathJoinSubstitution([
        FindPackageShare('kiss_icp_localisation'),
        'config',
        'fastlio2_velodyne.yaml',
    ])
    default_pgo_config = PathJoinSubstitution([
        FindPackageShare('kiss_icp_localisation'),
        'config',
        'fastlio2_pgo.yaml',
    ])

    lio_config = LaunchConfiguration('fastlio_config', default=default_lio_config)
    pgo_config = LaunchConfiguration('fastlio_pgo_config', default=default_pgo_config)
    use_loop_closure = LaunchConfiguration('use_fastlio_loop_closure', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    lio_node = Node(
        package='fastlio2',
        namespace='fastlio2',
        executable='lio_node',
        name='lio_node',
        output='screen',
        parameters=[{'config_path': lio_config, 'use_sim_time': use_sim_time}],
    )

    pgo_node = Node(
        package='pgo',
        namespace='pgo',
        executable='pgo_node',
        name='pgo_node',
        output='screen',
        parameters=[{'config_path': pgo_config, 'use_sim_time': use_sim_time}],
        condition=IfCondition(use_loop_closure),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'fastlio_config',
            default_value=default_lio_config,
            description='FASTLIO2 config YAML used for Velodyne + IMU integration.',
        ),
        DeclareLaunchArgument(
            'fastlio_pgo_config',
            default_value=default_pgo_config,
            description='PGO config YAML for FASTLIO2 loop closure.',
        ),
        DeclareLaunchArgument(
            'use_fastlio_loop_closure',
            default_value='false',
            description='Start PGO loop closure node with FASTLIO2 odometry/cloud.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for FASTLIO2 and optional PGO nodes.',
        ),
        lio_node,
        pgo_node,
    ])

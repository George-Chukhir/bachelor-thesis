"""Offline FASTLIO2 mapping pipeline for rosbag playback.

This launch is designed for dense map building from recorded bags:
- Starts FASTLIO2 under /fastlio2 namespace
- Optionally starts PGO loop closure node
- Optionally starts RViz with the PGO config
- Optionally starts ros2 bag play with only raw topics

Why filter bag topics:
Some bags already contain /fastlio2 outputs from previous runs. Replaying those
can mix old outputs with newly computed results, so this launch can replay only
raw sensor + TF streams.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _validate_bag_arguments(context):
    # Keep validation simple and explicit for launch args.
    play_bag_value = context.perform_substitution(LaunchConfiguration('play_bag')).strip().lower() == 'true'
    bag_loop_value = context.perform_substitution(LaunchConfiguration('bag_loop')).strip().lower() == 'true'
    bag_path_value = context.perform_substitution(LaunchConfiguration('bag_path')).strip()

    if play_bag_value and bag_loop_value:
        raise RuntimeError(
            "Arguments 'play_bag' and 'bag_loop' are mutually exclusive. "
            "Enable only one of them."
        )

    if (play_bag_value or bag_loop_value) and not bag_path_value:
        raise RuntimeError(
            "Missing required argument 'bag_path'. "
            "Pass a bag directory path when play_bag:=true or bag_loop:=true."
        )
    return []


def generate_launch_description() -> LaunchDescription:
    bringup_pkg = FindPackageShare('b2_thesis_fusion')

    default_fastlio_cfg = PathJoinSubstitution([
        bringup_pkg,
        'config',
        'fastlio2_velodyne.yaml',
    ])
    default_pgo_cfg = PathJoinSubstitution([
        bringup_pkg,
        'config',
        'fastlio2_pgo.yaml',
    ])
    default_rviz_cfg = PathJoinSubstitution([
        bringup_pkg,
        'rviz',
        'fastlio_offline_mapping.rviz',
    ])

    use_pgo = LaunchConfiguration('use_pgo', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    play_bag = LaunchConfiguration('play_bag', default='true')
    bag_path = LaunchConfiguration('bag_path', default='')
    bag_rate = LaunchConfiguration('bag_rate', default='1.0')
    bag_loop = LaunchConfiguration('bag_loop', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    fastlio_cfg = LaunchConfiguration('fastlio_config', default=default_fastlio_cfg)
    pgo_cfg = LaunchConfiguration('fastlio_pgo_config', default=default_pgo_cfg)
    rviz_cfg = LaunchConfiguration('rviz_config', default=default_rviz_cfg)


      ###
      #Static TFs
      ###

    static_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['--x', '0', '--y', '-0.02341', '--z', '0.04927', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                  '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        parameters=[{'use_sim_time': use_sim_time}]
        )
    
    static_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='correct_velodyne_tf',
        arguments=['0.3', '0.0', '0.4', '0.0', '0.0', '0.0', 'base_link', 'velodyne'],
        parameters=[{'use_sim_time': use_sim_time}]
        )
    


    fastlio_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_pkg,
                'launch',
                'fastlio2_velodyne_loop.launch.py',
            ])
        ),
        launch_arguments={
            'fastlio_config': fastlio_cfg,
            'fastlio_pgo_config': pgo_cfg,
            'use_fastlio_loop_closure': use_pgo,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    validate_bag_arguments = OpaqueFunction(function=_validate_bag_arguments)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # Raw-only bag playback avoids mixing previously recorded /fastlio2 outputs.
    bag_play = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            bag_path,
            '--clock',
            '--rate',
            bag_rate,
            '--topics',
            '/velodyne_points',
            '/imu/data_sync', # changed to /imu/data_sync
            #'/tf',
            #'/tf_static',
        ],
        output='screen',
        condition=IfCondition(play_bag),
    )

    bag_play_loop = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            bag_path,
            '--clock',
            '--rate',
            bag_rate,
            '--loop',
            '--topics',
            '/velodyne_points',
            '/imu/data_sync', # changed to /imu/data_sync
            #'/tf',
            #'/tf_static',
        ],
        output='screen',
        condition=IfCondition(bag_loop),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_pgo',
            default_value='true',
            description='Enable PGO node (loop closure and map optimization).',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with PGO visualization config.',
        ),
        DeclareLaunchArgument(
            'play_bag',
            default_value='true',
            description='Start ros2 bag play from this launch.',
        ),
        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description='Absolute path to bag directory (required when play_bag or bag_loop is true).',
        ),
        DeclareLaunchArgument(
            'bag_rate',
            default_value='1.0',
            description='ros2 bag play rate.',
        ),
        DeclareLaunchArgument(
            'bag_loop',
            default_value='false',
            description='Loop bag playback (implies play_bag behavior).',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from ros2 bag play.',
        ),
        DeclareLaunchArgument(
            'fastlio_config',
            default_value=fastlio_cfg,
            description='FASTLIO2 config file path.',
        ),
        DeclareLaunchArgument(
            'fastlio_pgo_config',
            default_value=pgo_cfg,
            description='PGO config file path.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_cfg,
            description='RViz config file path.',
        ),
        validate_bag_arguments,
        fastlio_stack,
        static_imu,
        static_velodyne,
        rviz_node,
        bag_play,
        bag_play_loop,
    ])

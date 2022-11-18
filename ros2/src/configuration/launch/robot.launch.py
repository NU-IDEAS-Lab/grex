from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "id", default_value=TextSubstitution(text="0")
        ),
        DeclareLaunchArgument(
            'name',
            default_value=["agent", LaunchConfiguration("id")]
        ),
        # Node(
        #     package="mgr_ce495cav",
        #     executable="sim_manager_static_avg",
        #     name="sim_manager",
        #     output="screen",
        #     emulate_tty=True,
        #     parameters=[
        #         {"algorithm_name": "Static Uniform, Disturbed"},
        #         {"agent_count": int(AGENT_COUNT)},
        #         {"goal_value": float(mean(range(AGENT_COUNT)))},
        #         {"goal_tolerance": 0.005},
        #         {"step_rate": 30.0},
        #     ]
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'localization_launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': LaunchConfiguration("name"),
                'map': '/home/anthony/dev/northwestern/aamas_environment/src/patrolling_sim/maps/DIAG_labs/DIAG_labs.yaml',
                # 'map': '/home/anthony/dev/northwestern/aamas_environment/src/patrolling_sim/maps/cumberland/cumberland.yaml',
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': LaunchConfiguration("name"),
            }.items()
        ),

        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     # respawn=use_respawn,
        #     # respawn_delay=2.0,
        #     parameters=[
        #         {"yaml_filename": "/home/anthony/dev/northwestern/aamas_environment/src/patrolling_sim/maps/DIAG_labs/DIAG_labs.yaml"},
        #     ],
        #     # arguments=['--ros-args', '--log-level', log_level],
        #     # remappings=remappings
        # ),
        # Node(
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     # respawn=use_respawn,
        #     # respawn_delay=2.0,
        #     # parameters=[configured_params],
        #     # arguments=['--ros-args', '--log-level', log_level],
        #     # remappings=remappings
        # ),
    ])
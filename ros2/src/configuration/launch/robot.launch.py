from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():
    return LaunchDescription([
        # Handle parameters.
        DeclareLaunchArgument(
            "id", default_value=TextSubstitution(text="0")
        ),
        DeclareLaunchArgument(
            'name', default_value=["agent", LaunchConfiguration("id")]
        ),
        DeclareLaunchArgument(
            'pose_x', default_value='0.0'
        ),
        DeclareLaunchArgument(
            'pose_y', default_value='0.0'
        ),
        DeclareLaunchArgument(
            'model_name', default_value='burger'
        ),
        DeclareLaunchArgument(
            'urdf_path', default_value=[FindPackageShare('turtlebot3_gazebo'), '/models/turtlebot3_', LaunchConfiguration("model_name"), '/model.sdf']
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false'
        ),

        # Start the navigation stack.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': LaunchConfiguration("name"),
                'use_namespace': 'True',
                'use_composition': 'False',
                'use_sim_time': 'True',
                'map': '/home/anthony/dev/aamas2023/src/patrolling_sim/maps/DIAG_labs/DIAG_labs.yaml',
                # 'map': '/home/anthony/dev/northwestern/aamas_environment/src/patrolling_sim/maps/DIAG_labs/DIAG_labs.yaml',
                # 'map': '/home/anthony/dev/northwestern/aamas_environment/src/patrolling_sim/maps/cumberland/cumberland.yaml',
            }.items()
        ),

        # Start RViz.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'rviz_launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': LaunchConfiguration("name"),
                'use_namespace': 'True',
            }.items(),
            condition = IfCondition(LaunchConfiguration("use_rviz")),
        ),

        GroupAction(
            actions=[
                PushRosNamespace(LaunchConfiguration("name")),
                SetRemap(dst='/tf', src=['/agent', LaunchConfiguration("id"), '/tf']),
                SetRemap(dst='/tf_static', src=['/agent', LaunchConfiguration("id"), '/tf_static']),
                # This node calls a Gazebo service to spawn the robot.
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'burger',
                        '-file', LaunchConfiguration("urdf_path"),
                        '-robot_namespace', LaunchConfiguration("name"),
                        '-x', LaunchConfiguration("pose_x"),
                        '-y', LaunchConfiguration("pose_y"),
                        '-z', '0.01'
                    ],
                ),
                # IncludeLaunchDescription(
                #     PythonLaunchDescriptionSource([
                #         PathJoinSubstitution([
                #             FindPackageShare('turtlebot3_gazebo'),
                #             'launch',
                #             'spawn_turtlebot3.launch.py'
                #         ])
                #     ]),
                #     # launch_arguments={}.items()
                # ),

                # Run the robot state publisher.
                IncludeLaunchDescription(
                    PathJoinSubstitution([
                        FindPackageShare('turtlebot3_gazebo'),
                        'launch',
                        'robot_state_publisher.launch.py'
                    ]),
                    launch_arguments={
                        'use_sim_time': 'True'
                    }.items()
                ),
            ]
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('nav2_bringup'),
        #             'launch',
        #             'localization_launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         # 'namespace': LaunchConfiguration("name"),
        #         'map': '/home/anthony/dev/aamas2023/src/patrolling_sim/maps/DIAG_labs/DIAG_labs.yaml',
        #         # 'map': '/home/anthony/dev/northwestern/aamas_environment/src/patrolling_sim/maps/DIAG_labs/DIAG_labs.yaml',
        #         # 'map': '/home/anthony/dev/northwestern/aamas_environment/src/patrolling_sim/maps/cumberland/cumberland.yaml',
        #     }.items()
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('nav2_bringup'),
        #             'launch',
        #             'navigation_launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'namespace': LaunchConfiguration("name"),
        #     }.items()
        # ),

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
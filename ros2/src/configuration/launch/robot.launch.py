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
            'map', default_value='/home/anthony/dev/northwestern/aamas_environment/src/patrolling_sim/maps/DIAG_labs/DIAG_labs.yaml'
        ),
        DeclareLaunchArgument(
            'model_name', default_value='waffle'
            # 'model_name', default_value='burger'
        ),
        DeclareLaunchArgument(
            # TEMPORARY - Looks like we will need to provide our own model files with Nav2's /tf namespace fix. TODO
            'urdf_path', default_value=[FindPackageShare('nav2_bringup'), '/worlds/', LaunchConfiguration("model_name"), '.model']
            # 'urdf_path', default_value=[FindPackageShare('turtlebot3_gazebo'), '/models/turtlebot3_', LaunchConfiguration("model_name"), '/model.sdf']
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false'
        ),

        # Start the navigation stack.
        GroupAction(
            actions=[
                # Manually remap the scan topic.
                SetRemap(src='/scan', dst=['/agent', LaunchConfiguration("id"), '/scan']),

                # Run the navigation stack.
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
                        'map': LaunchConfiguration("map"),
                        'params_file': PathJoinSubstitution([
                            FindPackageShare('configuration'),
                            'config',
                            'nav2_params.yaml'
                        ]),
                        # 'map': '/home/anthony/dev/aamas2023/src/patrolling_sim/maps/DIAG_labs/DIAG_labs.yaml',
                        # 'map': '/home/anthony/dev/northwestern/aamas_environment/src/patrolling_sim/maps/DIAG_labs/DIAG_labs.yaml',
                        # 'map': '/home/anthony/dev/northwestern/aamas_environment/src/patrolling_sim/maps/cumberland/cumberland.yaml',
                    }.items(),
                ),
            ]
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
                'rviz_config': PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'rviz',
                    'nav2_namespaced_view.rviz'
                ]),
            }.items(),
            condition = IfCondition(LaunchConfiguration("use_rviz")),
        ),

        GroupAction(
            actions=[
                PushRosNamespace(LaunchConfiguration("name")),
                SetRemap(src='/tf', dst=['/agent', LaunchConfiguration("id"), '/tf']),
                SetRemap(src='/tf_static', dst=['/agent', LaunchConfiguration("id"), '/tf_static']),
                # This node calls a Gazebo service to spawn the robot.
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', LaunchConfiguration("model_name"),
                        '-file', LaunchConfiguration("urdf_path"),
                        '-robot_namespace', LaunchConfiguration("name"),
                        '-x', LaunchConfiguration("pose_x"),
                        '-y', LaunchConfiguration("pose_y"),
                        '-z', '0.01'
                    ],
                ),

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
    ])
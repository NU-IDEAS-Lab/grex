from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap, SetParameter

from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    use_respawn = False
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    use_sim_time = True
    log_level = "info"
    autostart = True
    namespace = LaunchConfiguration('name')
    params_file = PathJoinSubstitution([
        FindPackageShare('simulation_base'),
        'config',
        'nav2_params.yaml'
    ])
    lifecycle_nodes = ['map_server', 'amcl']

    param_substitutions = {
        'use_sim_time': str(use_sim_time),
        'yaml_filename': LaunchConfiguration("map_file")}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

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
            'map', default_value='cumberland'
        ),
        DeclareLaunchArgument(
            'map_file', default_value=[FindPackageShare("simulation_base"), "/models/maps/", LaunchConfiguration("map"), "/", LaunchConfiguration("map"), ".yaml"]
        ),
        DeclareLaunchArgument(
            'model_name', default_value='waffle'
        ),
        DeclareLaunchArgument(
            # TEMPORARY - Looks like we will need to provide our own model files with Nav2's /tf namespace fix. TODO
            'urdf_path', default_value=[FindPackageShare('nav2_bringup'), '/worlds/', LaunchConfiguration("model_name"), '.model']
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false'
        ),

        # Start the navigation stack.
        GroupAction(
            actions=[
                # Manually remap the scan topic.
                SetRemap(src='/scan', dst=['/agent', LaunchConfiguration("id"), '/scan']),

                # Set miscellaneous parameters.
                SetParameter(
                    name="amcl/set_initial_pose",
                    value=True
                ),
                SetParameter(
                    name="amcl/initial_pose.x",
                    value=LaunchConfiguration("pose_x")
                ),
                SetParameter(
                    name="amcl/initial_pose.y",
                    value=LaunchConfiguration("pose_y")
                ),

                # Run the navigation stack.
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings
                ),
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=remappings
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    arguments=['--ros-args', '--log-level', log_level],
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': autostart},
                                {'node_names': lifecycle_nodes}]
                ),
                # IncludeLaunchDescription(
                #     PythonLaunchDescriptionSource([
                #         PathJoinSubstitution([
                #             FindPackageShare('nav2_bringup'),
                #             'launch',
                #             'bringup_launch.py'
                #         ])
                #     ]),
                #     launch_arguments={
                #         'namespace': LaunchConfiguration("name"),
                #         'use_namespace': 'True',
                #         'use_composition': 'False',
                #         'use_sim_time': 'True',
                #         'map': LaunchConfiguration("map_file"),
                #         'params_file': PathJoinSubstitution([
                #             FindPackageShare('simulation_base'),
                #             'config',
                #             'nav2_params.yaml'
                #         ]),
                #     }.items(),
                # ),
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
                        '-entity', LaunchConfiguration("name"),
                        '-file', LaunchConfiguration("urdf_path"),
                        '-robot_namespace', LaunchConfiguration("name"),
                        '-timeout', '500.0',
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
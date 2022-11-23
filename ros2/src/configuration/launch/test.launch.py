from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace, SetRemap

from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from statistics import mean

# import launch

def generate_launch_description():

    AGENT_COUNT = 1

    # Get the gazebo world model.
    # TODO: Replace with generated versions of the mapserver maps.
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
        # 'empty_world.world'
    )

    agents = []
    for agent in range(AGENT_COUNT):
        agents.append(GroupAction(
            actions=[
                # THE NAV STACK MIGHT ACTUALLY NOT WANT THIS NAMESPACE SET
                # PushRosNamespace('agent' + str(agent)),
                # SetRemap(dst='/tf', src='/agent' + str(agent) + '/tf'),
                # SetRemap(dst='/tf_static', src='/agent' + str(agent) + '/tf_static'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('configuration'),
                            'launch',
                            'robot.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        "id": str(agent),
                        "use_rviz": LaunchConfiguration("use_rviz"),
                    }.items()
                )
            ]
        ))

    return LaunchDescription([
        # Arguments.
        DeclareLaunchArgument(
            'use_rviz', default_value='false'
        ),

        # Agent nodes.
        *agents,

        # Gazebo simulation server.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gzserver.launch.py'
                ])
            ]),
            launch_arguments={
                'world': world
            }.items()
        ),

        # Gazebo client (GUI).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gzclient.launch.py'
                ])
            ])
        ),

        # Event handlers.
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=sim_mgr,
        #         on_exit=[
        #             LogInfo(msg="Shutdown initiated."),
        #             EmitEvent(event=Shutdown(
        #                 reason='Simulation completed closed'))
        #         ]
        #     )
        # ),
    ])
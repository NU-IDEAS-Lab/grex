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

    agents = []
    for agent in range(AGENT_COUNT):
        agents.append(
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
                    "map": LaunchConfiguration("map"),
                    "pose_x": "35.0",
                    "pose_y": "22.0",
                }.items()
            )
        )

    return LaunchDescription([
        # Arguments.
        DeclareLaunchArgument(
            'use_rviz', default_value='false'
        ),
        DeclareLaunchArgument(
            'map', default_value='cumberland'
        ),
        DeclareLaunchArgument(
            'gazebo_world_file', default_value=[FindPackageShare("configuration"), "/models/maps/", LaunchConfiguration("map"), "/model.sdf"]
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
                'world': LaunchConfiguration("gazebo_world_file"),
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
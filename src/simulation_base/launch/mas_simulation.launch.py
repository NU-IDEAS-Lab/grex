from launch import LaunchDescription, LaunchContext
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace, SetRemap

from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from statistics import mean

# import launch

def generate_agents(context: LaunchContext, agent_count_subst):
    ''' Generates the list of agent launch descriptions. '''

    # Convert agent count to integer.
    agent_count = int(context.perform_substitution(agent_count_subst))
    agents = []
    for agent in range(agent_count):
        agents += [
            LogInfo(msg=TextSubstitution(text="Creating agent " + str(agent))),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('simulation_base'),
                        'launch',
                        'robot.launch.py'
                    ])
                ]),
                launch_arguments={
                    "id": str(agent),
                    "name": "agent" + str(agent),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "map": LaunchConfiguration("map"),
                    "pose_x": str(35.0 + agent),
                    "pose_y": "22.0",
                }.items()
            )
        ]
    
    return agents

def generate_launch_description():
    ''' Generates the overall launch description. '''

    return LaunchDescription([
        # Arguments.
        DeclareLaunchArgument(
            'agent_count', default_value='1'
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false'
        ),
        DeclareLaunchArgument(
            'use_gzclient', default_value='true'
        ),
        DeclareLaunchArgument(
            'map', default_value='cumberland'
        ),
        DeclareLaunchArgument(
            'gazebo_world_file', default_value=[FindPackageShare("simulation_base"), "/models/maps/", LaunchConfiguration("map"), "/model.sdf"]
        ),

        # Gazebo simulation server.
        GroupAction(
            actions=[
                SetEnvironmentVariable(
                    name="DISPLAY",
                    value=":0"
                ),
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
            ]
        ),

        # Gazebo client (GUI).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gzclient.launch.py'
                ])
            ]),
            condition = IfCondition(LaunchConfiguration("use_gzclient")),
        ),

        # Agent nodes.
        OpaqueFunction(
            function=generate_agents,
            args=[LaunchConfiguration('agent_count')]
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
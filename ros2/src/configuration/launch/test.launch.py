from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os
from statistics import mean

# import launch

def generate_launch_description():

    AGENT_COUNT = 1

    # brokerConfig = os.path.join(
    #     get_package_share_directory('configuration'),
    #     'config',
    #     'demo_static_average_broker.yml'
    #     # 'demo_static_average_broker_nodrop.yml'
    # )

    # sim_mgr = Node(
    #         package="mgr_ce495cav",
    #         executable="sim_manager_static_avg",
    #         name="sim_manager",
    #         output="screen",
    #         emulate_tty=True,
    #         parameters=[
    #             {"algorithm_name": "Static Uniform, Disturbed"},
    #             {"agent_count": int(AGENT_COUNT)},
    #             {"goal_value": float(mean(range(AGENT_COUNT)))},
    #             {"goal_tolerance": 0.005},
    #             {"step_rate": 30.0},
    #         ]
    #     )

    agents = []
    for agent in range(AGENT_COUNT):
        agents.append(GroupAction(
            actions=[
                # push_ros_namespace to set namespace of included nodes
                PushRosNamespace('agent' + str(agent)),
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
                    }.items()
                )
            ]
        ))

    return LaunchDescription([
        # Agent nodes.
        *agents,


        # Control nodes.
        # Node(
        #     package="mas_broker",
        #     executable="broker",
        #     name="broker",
        #     output="screen",
        #     emulate_tty=True,
        #     parameters=[brokerConfig]
        # ),
        # sim_mgr,

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
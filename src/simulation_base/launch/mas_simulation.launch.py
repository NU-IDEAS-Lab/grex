from launch import LaunchDescription, LaunchContext
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace, SetRemap

from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch_ros_manager_py.actions import LaunchManagementServiceNode
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from statistics import mean



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
                        'agent',
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
            'use_agents', default_value='true'
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false'
        ),
        DeclareLaunchArgument(
            'headless', default_value='false'
        ),
        DeclareLaunchArgument(
            'map', default_value='cumberland'
        ),
        DeclareLaunchArgument(
            'gazebo_world_file', default_value=[FindPackageShare("simulation_base"), "/models/maps/", LaunchConfiguration("map"), "/model.sdf"]
        ),
        DeclareLaunchArgument(
            'simulator_launch_file', default_value=[FindPackageShare("simulation_base"), "/launch/simulator/gazebo.launch.yaml"]
        ),

        # Launch the management service node, which allows us to control the launch process via ROS service calls.
        LaunchManagementServiceNode(),

        # Launch the simulator.
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                LaunchConfiguration("simulator_launch_file")
            )
        ),

        # Agent nodes.
        OpaqueFunction(
            function=generate_agents,
            args=[LaunchConfiguration('agent_count')],
            condition = IfCondition(LaunchConfiguration("use_agents"))
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
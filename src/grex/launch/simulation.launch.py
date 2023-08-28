from launch import LaunchDescription, LaunchContext
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace, SetParameter, SetRemap

from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch_ros_manager_py.actions import LaunchManagementServiceNode
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from statistics import mean



def generate_agents(context: LaunchContext, agent_count_subst, sim_subst):
    ''' Generates the list of agent launch descriptions. '''

    # Parse the simulator type.
    sim = str(context.perform_substitution(sim_subst))
    if sim == "gazebo":
        simulator_agent_integration_launch_file = [FindPackageShare("grex"), "/launch/simulator/gazebo/agent.launch.yaml"]
    elif sim == "flatland":
        simulator_agent_integration_launch_file = [FindPackageShare("grex"), "/launch/simulator/flatland/agent.launch.py"]
    else:
        print("Unknown simulator type: " + sim + ". Defaulting to gazebo.")
        simulator_agent_integration_launch_file = [FindPackageShare("grex"), "/launch/simulator/gazebo/agent.launch.yaml"]

    # Convert agent count to integer.
    agent_count = int(context.perform_substitution(agent_count_subst))
    agents = []
    for agent in range(agent_count):
    
        # default agent namespace
        agentNs = "agent" + str(agent)

        agents += [
            LogInfo(msg=TextSubstitution(text="Creating agent " + str(agent))),

            # Set namespace.
            GroupAction(
                actions=[
                    PushRosNamespace(agentNs),

                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            LaunchConfiguration("agent_launch_file")
                        ),
                        launch_arguments={
                            "id": str(agent),
                            "namespace": agentNs,
                            "use_rviz": LaunchConfiguration("use_rviz"),
                            "map": LaunchConfiguration("map"),
                            "initial_pos_x": str(35.0 + agent),
                            "initial_pos_y": "22.0",
                            "sim": sim,
                            "simulator_agent_integration_launch_file": simulator_agent_integration_launch_file,
                        }.items()
                    )
                ]
            ),
        ]
    
    return agents

def launch_simulation(context: LaunchContext, sim_subst):
    sim = str(context.perform_substitution(sim_subst))
    if sim == "gazebo":
        simulator_launch_file = [FindPackageShare("grex"), "/launch/simulator/gazebo/simulator.launch.yaml"]
    elif sim == "flatland":
        simulator_launch_file = [FindPackageShare("grex"), "/launch/simulator/flatland/simulator.launch.yaml"]
    else:
        print("Unknown simulator type: " + sim + ". Defaulting to gazebo.")
        simulator_launch_file = [FindPackageShare("grex"), "/launch/simulator/gazebo/simulator.launch.yaml"]

    simulator = []
    simulator += [
        LogInfo(msg=TextSubstitution(text="Launching simulator: " + sim)),

        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(
                        simulator_launch_file
                    )
                )
            ]
        )
    ]
    # Launch the simulator.
    return simulator

def generate_launch_description():
    ''' Generates the overall launch description. '''

    return LaunchDescription([
        # Arguments.
        DeclareLaunchArgument(
            'sim', default_value='gazebo'
        ),
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
            'gazebo_world_file', default_value=[FindPackageShare("grex"), "/models/maps/", LaunchConfiguration("map"), "/model.sdf"]
        ),
        DeclareLaunchArgument(
            'flatland_world_file', default_value=[FindPackageShare("grex"), "/models/maps/", LaunchConfiguration("map"), "/", LaunchConfiguration("map"), "_flatland.yaml"]
        ),
        DeclareLaunchArgument(
            'simulator_launch_file', default_value=[FindPackageShare("grex"), "/launch/simulator/gazebo/simulator.launch.yaml"]
        ),
        DeclareLaunchArgument(
            'simulator_agent_integration_launch_file', default_value=[FindPackageShare("grex"), "/launch/simulator/gazebo/agent.launch.yaml"]
        ),
        DeclareLaunchArgument(
            'agent_launch_file', default_value=[FindPackageShare("grex"), "/launch/agent/example/agent.launch.yaml"]
        ),
        
        # Enable simulation time throughout Grex.
        SetParameter(
            name = "use_sim_time",
            value = "True"
        ),

        # Launch the management service node, which allows us to control the launch process via ROS service calls.
        LaunchManagementServiceNode(),

        OpaqueFunction(
            function=launch_simulation,
            args=[LaunchConfiguration('sim')]
        ),

        # Agent nodes.
        OpaqueFunction(
            function=generate_agents,
            args=[LaunchConfiguration('agent_count'), LaunchConfiguration('sim')],
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
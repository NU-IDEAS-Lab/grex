# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



##
## NOTE: This file is derived from the nav2 project:
##          https://github.com/ros-planning/navigation2/blob/7657f2f37659808884ae6a708f3d51054af6355c/nav2_bringup/launch/bringup_launch.py
##       Modifications by Anthony Goeckner for use by the IDEAS Lab at Northwestern University.
##


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter
from nav2_common.launch import RewrittenYaml

def generate_bringup(context: LaunchContext, sim_subst, launch_dir):
    sim = str(context.perform_substitution(sim_subst))
    remappings = []

    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'yaml_filename': LaunchConfiguration('map'),
        'amcl.ros__parameters.base_frame_id': [LaunchConfiguration('namespace'), '/base_link'],
        'amcl.ros__parameters.odom_frame_id': [LaunchConfiguration('namespace'), '/odom'],
        'bt_navigator.ros__parameters.robot_base_frame': [LaunchConfiguration('namespace'), '/base_link'],
        'local_costmap.local_costmap.ros__parameters.global_frame': [LaunchConfiguration('namespace'), '/odom'],
        'local_costmap.local_costmap.ros__parameters.robot_base_frame': [LaunchConfiguration('namespace'), '/base_link'],
        'local_costmap.local_costmap.ros__parameters.voxel_layer.scan.sensor_frame': [LaunchConfiguration('namespace'), '/base_scan'],
        'global_costmap.global_costmap.ros__parameters.robot_base_frame': [LaunchConfiguration('namespace'), '/base_link'],
        'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.sensor_frame': [LaunchConfiguration('namespace'), '/base_scan'],
        'behavior_server.ros__parameters.global_frame': [LaunchConfiguration('namespace'), '/odom'],
        'behavior_server.ros__parameters.robot_base_frame': [LaunchConfiguration('namespace'), '/base_link'],
        'smoother_server.ros__parameters.robot_base_frame': [LaunchConfiguration('namespace'), '/base_link'],
    }

    if sim == 'gazebo':
        param_substitutions['amcl.ros__parameters.global_frame_id'] = [LaunchConfiguration('namespace'), '/map']
        param_substitutions['bt_navigator.ros__parameters.global_frame'] = [LaunchConfiguration('namespace'), '/map']
        param_substitutions['global_costmap.global_costmap.ros__parameters.global_frame'] = [LaunchConfiguration('namespace'), '/map']
        param_substitutions['map_server.ros__parameters.frame_id'] = [LaunchConfiguration('namespace'), '/map']
   
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=LaunchConfiguration('namespace'),
        param_rewrites=param_substitutions,
        convert_types=True)
    
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(LaunchConfiguration('use_namespace')),
            namespace=LaunchConfiguration('namespace')),

        Node(
            condition=IfCondition(LaunchConfiguration('use_composition')),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': LaunchConfiguration('autostart')}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(LaunchConfiguration('slam')),
            launch_arguments={'namespace': LaunchConfiguration('namespace'),
                              'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'autostart': LaunchConfiguration('autostart'),
                              'use_respawn': LaunchConfiguration('use_respawn'),
                              'configured_params_file': configured_params}.items()),

        GroupAction(
            actions=[
                # Set initial pose.
                SetParameter(
                    name="set_initial_pose",
                    value=True
                ),
                SetParameter(
                    name="initial_pose.x",
                    value=LaunchConfiguration("pose_x")
                ),
                SetParameter(
                    name="initial_pose.y",
                    value=LaunchConfiguration("pose_y")
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                            'localization_launch.py')),
                    condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('slam')])),
                    launch_arguments={'namespace': LaunchConfiguration('namespace'),
                                    'map': LaunchConfiguration('map'),
                                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                                    'autostart': LaunchConfiguration('autostart'),
                                    'configured_params_file': configured_params,
                                    'use_composition': LaunchConfiguration('use_composition'),
                                    'use_respawn': LaunchConfiguration('use_respawn'),
                                    'container_name': 'nav2_container'}.items()
                ),
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': LaunchConfiguration('namespace'),
                              'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'autostart': LaunchConfiguration('autostart'),
                              'configured_params_file': configured_params,
                              'use_composition': LaunchConfiguration('use_composition'),
                              'use_respawn': LaunchConfiguration('use_respawn'),
                              'container_name': 'nav2_container'}.items()),
    ])

    return [bringup_cmd_group]
    
def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('grex_agent_nav')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    sim = LaunchConfiguration('sim')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    # remappings = [('/tf', 'tf'),
    #               ('/tf_static', 'tf_static')]
    remappings = []

    # Create our own temporary YAML files that include substitutions
    
    declare_sim = DeclareLaunchArgument(
        'sim',
        default_value='gazebo',
        description='flatland or gazebo')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    # Initial position given to the localization system.
    declare_pose_x_cmd = DeclareLaunchArgument(
            'pose_x', default_value='0.0')
    declare_pose_y_cmd = DeclareLaunchArgument(
            'pose_y', default_value='0.0')

    # Specify the actions
    bringup_cmd_group = OpaqueFunction(
        function=generate_bringup,
        args=[LaunchConfiguration('sim'), launch_dir]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_sim)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_pose_x_cmd)
    ld.add_action(declare_pose_y_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld

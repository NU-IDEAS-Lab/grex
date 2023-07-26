from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'id', default_value='0'
        ),
        DeclareLaunchArgument(
            'namespace', default_value=['agent', LaunchConfiguration('id')]
        ),
        DeclareLaunchArgument(
            'initial_pos_x', default_value='0.0'
        ),
        DeclareLaunchArgument(
            'initial_pos_y', default_value='0.0'
        ),
        DeclareLaunchArgument(
            'initial_pos_theta', default_value='0.0'
        ),

        DeclareLaunchArgument(
            '_odom_frame_concat', default_value=[LaunchConfiguration('namespace'), '/odom']
        ),

        Node(
            name='tf_map_to_odom',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', LaunchConfiguration('_odom_frame_concat')],
        ),

        ExecuteProcess(
            cmd=[[
                FindExecutable(name="ros2"),
                " service ",
                "call ",
                "/spawn_model ",
                "flatland_msgs/srv/SpawnModel ",
                "\"{yaml_path: '", PathJoinSubstitution([FindPackageShare("grex"), "models/robots/turtlebot.model.yaml"]),
                "', name: 'turtlebot_", LaunchConfiguration('id'), "', ns: ", LaunchConfiguration('namespace'), ", pose: ",
                "{x: ",  LaunchConfiguration('initial_pos_x'), ", y: ", LaunchConfiguration('initial_pos_y'), ", theta: ", LaunchConfiguration('initial_pos_theta'), "}}\"", 
            ]],
            shell=True,
        )
    ])
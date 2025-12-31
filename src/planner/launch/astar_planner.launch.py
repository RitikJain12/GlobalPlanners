from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='planner',
            executable='test_planner_a_star',
            # name='astar_planner',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('planner'), 'config', 'astar_params.yaml'])
            ],
        ),
    ])

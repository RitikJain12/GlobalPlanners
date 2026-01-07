from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    planner_type = LaunchConfiguration("planner")

    config_file = PathJoinSubstitution([
        FindPackageShare("planner"),
        "config",
        PythonExpression([
            "'astar_params.yaml' if '", planner_type,
            "' == 'AStar' else ('hybrid_a_star.yaml' if '",
            planner_type, "' == 'HybridA' else '')"
        ])
    ])

    return LaunchDescription([
        Node(
            package='planner',
            executable='test_planner_a_star',
            name='astar_planner',
            output='screen',
            parameters=[config_file],
        ),
    ])

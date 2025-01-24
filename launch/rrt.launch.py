from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    Command,
)
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_bounds",
                default_value="[10.0, 10.0, 10.0]",
                description="The start position for RRT",
            ),
            DeclareLaunchArgument(
                "start_coord",
                default_value="[0.0, 0.0, 0.0]",
                description="The start position for RRT",
            ),
            DeclareLaunchArgument(
                "goal_coord",
                default_value="[0.0, 0.0, 0.0]",
                description="The start position for RRT",
            ),
            DeclareLaunchArgument(
                "step_size",
                default_value="1.0",
                description="Default step size along unit vector in RRT for new nodes",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("rrt3d"),
                            "config",
                            "rrt3d.rviz",
                        ]
                    ),
                ],
            ),
            Node(
                package="rrt3d",
                executable="rrt3d",
                output="screen",
                parameters=[
                    {
                        "start_coord": LaunchConfiguration("start_coord"),
                        "goal_coord": LaunchConfiguration("goal_coord"),
                        "world_bounds": LaunchConfiguration("world_bounds"),
                    }
                ],
            ),
        ]
    )

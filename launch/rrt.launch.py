from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
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
                default_value="[0.0]",
                description="The start position for RRT",
            ),
            DeclareLaunchArgument(
                "goal_coord",
                default_value="[0.0]",
                description="The start position for RRT",
            ),
            DeclareLaunchArgument(
                "step_size",
                default_value="1.0",
                description="Default step size along unit vector in RRT for new nodes",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether or not to use rviz"
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
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("use_rviz"), "' == 'true'"]
                    )
                ),
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    Command,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.conditions import IfCondition
import launch


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Specify whether rviz is launched.",
            ),
            DeclareLaunchArgument(
                name="use_jsp",
                default_value="true",
                choices=["true", "false"],
                description="Specify whether joint_state_publisher is used.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                ExecutableInPackage("xacro", "xacro"),
                                " ",
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("nuturtle_description"),
                                        "urdf/turtlebot3_burger.urdf.xacro",
                                    ]
                                ),
                            ]
                        )
                    }
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("use_jsp"), "' == 'true'"]
                    )
                ),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nuturtle_description"),
                            "config/basic_purple.rviz",
                        ]
                    ),
                ],
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("use_rviz"), "' == 'true'"]
                    )
                ),
                on_exit=launch.actions.Shutdown(),
            ),
        ]
    )

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EqualsSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, LaunchConfigurationEquals


def generate_launch_description():

    urdf_file_name = 'urdf/turtlebot3_burger.urdf.xacro'
    urdf = os.path.join(
        get_package_share_directory('nuturtle_description'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices = ['true', 'false'],
            description='Specify whether rviz is launched.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
        # Node(
        #     package='nuturtle_description',
        #     executable='state_publisher',
        #     name='state_publisher',
        #     output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_rviz'), "' == 'true'"]))
        )

        
    ])
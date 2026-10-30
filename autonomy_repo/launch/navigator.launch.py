#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # relay goal poses published from RVIZ onto /cmd_nav
        Node(
            executable="rviz_goal_relay.py",
            package="asl_tb3_lib",
            parameters=[{"output_channel": "/cmd_nav"}],
        ),

        # publish the robot state on /state
        Node(
            executable="state_publisher.py",
            package="asl_tb3_lib",
        ),

        # our navigator node
        Node(
            executable="navigator.py",
            package="autonomy_repo",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),

        # RVIZ
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("asl_tb3_sim"), "launch", "rviz.launch.py",
                ])
            ),
            launch_arguments={
                "config": PathJoinSubstitution([
                    FindPackageShare("autonomy_repo"), "rviz", "default.rviz",
                ]),
                "use_sim_time": use_sim_time,
            }.items(),
        ),
    ])

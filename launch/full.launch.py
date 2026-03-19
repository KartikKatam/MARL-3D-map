"""Full pipeline launch: sim + SLAM with a delay.

Starts Isaac Sim first, then launches cuVSLAM + map_saver after 10 seconds
to give the sim time to initialize and start publishing.

For single-machine testing only. In production, containers are started separately.

    ros2 launch drone_slam full.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("drone_slam")

    output_dir_arg = DeclareLaunchArgument(
        "output_dir", default_value="output/maps"
    )

    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false"
    )

    # Launch sim immediately
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "sim.launch.py"])
        ),
        launch_arguments={"headless": LaunchConfiguration("headless")}.items(),
    )

    # Launch SLAM pipeline after 10s delay
    slam_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_share, "launch", "slam.launch.py"])
                ),
                launch_arguments={
                    "output_dir": LaunchConfiguration("output_dir"),
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        output_dir_arg,
        headless_arg,
        sim_launch,
        slam_launch,
    ])

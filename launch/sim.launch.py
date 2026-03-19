"""Launch the Isaac Sim standalone script.

In practice, the sim is usually launched manually inside its container.
This launch file exists for single-machine testing convenience.

    ros2 launch drone_slam sim.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    isaac_sim_python_arg = DeclareLaunchArgument(
        "isaac_sim_python",
        default_value="/isaac-sim/python.sh",
        description="Path to Isaac Sim's python.sh",
    )

    sim_app_path_arg = DeclareLaunchArgument(
        "sim_app_path",
        default_value="drone_slam/sim_app.py",
        description="Path to sim_app.py",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Isaac Sim in headless mode",
    )

    config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="config",
        description="Path to config directory",
    )

    sim_process = ExecuteProcess(
        cmd=[
            LaunchConfiguration("isaac_sim_python"),
            LaunchConfiguration("sim_app_path"),
            "--config-dir", LaunchConfiguration("config_dir"),
            # Conditionally add --headless based on arg
            # For simplicity, always pass it and let argparse handle
        ],
        output="screen",
    )

    return LaunchDescription([
        isaac_sim_python_arg,
        sim_app_path_arg,
        headless_arg,
        config_dir_arg,
        sim_process,
    ])

"""Launch cuVSLAM + map_saver for the drone SLAM pipeline.

Run inside the Isaac ROS container:
    ros2 launch drone_slam slam.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="output/maps",
        description="Directory to save PLY map files",
    )

    cuVSLAM_config_arg = DeclareLaunchArgument(
        "cuVSLAM_config",
        default_value="config/cuVSLAM.yaml",
        description="Path to cuVSLAM parameter file",
    )

    # cuVSLAM as ComposableNode in a multi-threaded container
    cuVSLAM_container = ComposableNodeContainer(
        name="cuVSLAM_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="isaac_ros_visual_slam",
                plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
                name="visual_slam",
                remappings=[
                    ("visual_slam/image_0", "front_stereo_camera/left/image_rect_color"),
                    ("visual_slam/camera_info_0", "front_stereo_camera/left/camera_info"),
                    ("visual_slam/image_1", "front_stereo_camera/right/image_rect_color"),
                    ("visual_slam/camera_info_1", "front_stereo_camera/right/camera_info"),
                ],
                parameters=[LaunchConfiguration("cuVSLAM_config")],
            ),
        ],
        output="screen",
    )

    # map_saver node
    map_saver_node = Node(
        package="drone_slam",
        executable="map_saver",
        name="map_saver",
        parameters=[
            {"use_sim_time": True},
            {"output_dir": LaunchConfiguration("output_dir")},
        ],
        output="screen",
    )

    return LaunchDescription([
        output_dir_arg,
        cuVSLAM_config_arg,
        cuVSLAM_container,
        map_saver_node,
    ])

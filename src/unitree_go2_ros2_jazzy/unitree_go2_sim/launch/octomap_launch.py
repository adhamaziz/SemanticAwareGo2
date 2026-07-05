import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    unitree_go2_sim = get_package_share_directory("unitree_go2_sim")
    octomap_params_file = os.path.join(unitree_go2_sim, "config", "octomap_params.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    octomap_server_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server_node",
        output="screen",
        parameters=[
            octomap_params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cloud_in", "/velodyne_points/points"),
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            octomap_server_node,
        ]
    )

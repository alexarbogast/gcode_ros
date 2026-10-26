from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    default_rviz_config = os.path.join(
        get_package_share_directory("gcode_rviz"), "rviz", "opengcode.rviz"
    )

    rvizconfig_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=default_rviz_config,
        description="Path to RViz config file",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="open_gcode_plugin_example",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        output="screen",
    )

    return LaunchDescription([rvizconfig_arg, rviz_node])

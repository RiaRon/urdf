"""Show one LEAP Hand in RViz with joint sliders.

    ros2 launch leap_hand_description display.launch.py side:=left
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context: LaunchContext):
    share = get_package_share_directory("leap_hand_description")
    side = LaunchConfiguration("side").perform(context)
    description = xacro.process_file(
        os.path.join(share, "urdf", "leap_hand.urdf.xacro"), mappings={"side": side}
    ).toxml()
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(share, "rviz", "leap_hand.rviz")]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="right", description="left or right"),
        OpaqueFunction(function=_nodes),
    ])

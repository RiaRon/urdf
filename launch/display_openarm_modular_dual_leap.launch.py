"""Show OpenArm dual + LEAP Hands (assemblies/openarm_modular_dual_leap.xacro) in RViz.

    source /opt/ros/jazzy/setup.bash
    colcon build --base-paths vendor/openarm_description leap_hand_description
    source install/setup.bash
    ros2 launch launch/display_openarm_modular_dual_leap.launch.py
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

ASSEMBLY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assemblies",
                        "openarm_modular_dual_leap.xacro")


def generate_launch_description():
    description = xacro.process_file(ASSEMBLY).toxml()
    rviz = os.path.join(get_package_share_directory("openarm_description"), "rviz", "bimanual.rviz")
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz]),
    ])

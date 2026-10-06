"""Mock-only OpenArm + RH56F1 visualization launch.

This launch intentionally starts no ros2_control node, driver, controller,
CAN interface, or RH56F1 transport.  Joint states come only from the standard
joint_state_publisher (GUI or headless).
"""

from pathlib import Path
import sys

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))

from openarm_rh56f1_visualization import build_robot_description  # noqa: E402


def _spawn_nodes(context: LaunchContext):
    hand_configuration = context.perform_substitution(
        LaunchConfiguration("hand_configuration")
    )
    robot_description = build_robot_description(hand_configuration)
    use_gui = LaunchConfiguration("use_gui")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="openarm_rh56f1_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="openarm_rh56f1_joint_state_publisher_gui",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(use_gui),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="openarm_rh56f1_joint_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=UnlessCondition(use_gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="openarm_rh56f1_rviz",
            arguments=["-d", str(REPO_ROOT / "rviz/openarm_rh56f1.rviz")],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hand_configuration",
                default_value="both",
                choices=["arm_only", "left", "right", "both"],
                description="RH56F1 model attachment selection",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                choices=["true", "false"],
                description="Use joint_state_publisher_gui instead of headless mock states",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Start RViz2",
            ),
            OpaqueFunction(function=_spawn_nodes),
        ]
    )

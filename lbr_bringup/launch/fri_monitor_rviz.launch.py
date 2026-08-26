from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.rviz import RVizMixin


def generate_launch_description() -> LaunchDescription:
    model = LBRDescriptionMixin.arg_model()
    robot_name = LBRDescriptionMixin.arg_robot_name(default_value="iiwa7")

    robot_description = {
        "robot_description": Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("wsg50_description"),
                "urdf",
                "iiwa7_wsg50.xacro",
            ]),
            " robot_name:=iiwa7 mode:=mock",
        ])
    }

    gripper_ip = DeclareLaunchArgument(
        "gripper_ip", default_value="192.168.1.160",
        description="WSG50 controller IP address.")
    gripper_port = DeclareLaunchArgument(
        "gripper_port", default_value="1501",
        description="WSG50 TCP port.")

    return LaunchDescription(
        [
            model,
            robot_name,
            gripper_ip,
            gripper_port,
            Node(
                package="lbr_demos_cpp",
                executable="fri_monitor",
                name="fri_monitor",
                namespace="iiwa7",
                output="screen",
                parameters=[
                    {"controller_ip": "172.31.1.147", "port": 30200,
                     "robot_name": "iiwa7"}
                ],
            ),
            Node(
                package="wsg50_driver",
                executable="wsg50_gripper_driver_node",
                name="wsg50",
                output="screen",
                parameters=[
                    {"gripper_ip": LaunchConfiguration("gripper_ip"),
                     "port": LaunchConfiguration("gripper_port")}
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace="iiwa7",
                output="screen",
                parameters=[robot_description],
                remappings=[("robot_description", "robot_description")],
            ),
            RVizMixin.node_rviz(
                remappings=[("/lbr/robot_description", "/iiwa7/robot_description")]
            ),
        ]
    )

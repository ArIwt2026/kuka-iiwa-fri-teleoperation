from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    # This is the ROS namespace used by the old workcell launch (/iiwa7).
    # The robot model keeps the "lbr" frame/joint prefix expected by the
    # installed controller and force-torque configuration.
    robot_namespace = LaunchConfiguration("robot_namespace")

    system_config_path = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration("sys_cfg_pkg")), LaunchConfiguration("sys_cfg")]
    )
    initial_joint_positions_path = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("init_jnt_pos_pkg")),
            LaunchConfiguration("init_jnt_pos"),
        ]
    )
    home_config_path = PathJoinSubstitution(
        [FindPackageShare(LaunchConfiguration("home_cfg_pkg")), LaunchConfiguration("home_cfg")]
    )
    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                PathJoinSubstitution(
                    [FindPackageShare("wsg50_description"), "urdf", "iiwa7_wsg50.xacro"]
                ),
                " robot_name:=lbr mode:=hardware system_config_path:=",
                system_config_path,
                " initial_joint_positions_path:=",
                initial_joint_positions_path,
            ]
        )
    }

    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description,
        robot_name=robot_namespace,
        use_sim_time=False,
    )
    ros2_control_node = LBRROS2ControlMixin.node_ros2_control(
        robot_name=robot_namespace,
        use_sim_time=False,
        robot_description=robot_description,
        additional_parameters=[home_config_path],
    )

    spawners = [
        LBRROS2ControlMixin.node_controller_spawner(
            robot_name=robot_namespace, controller="joint_state_broadcaster"
        ),
        LBRROS2ControlMixin.node_controller_spawner(
            robot_name=robot_namespace, controller="force_torque_broadcaster"
        ),
        LBRROS2ControlMixin.node_controller_spawner(
            robot_name=robot_namespace, controller="lbr_state_broadcaster"
        ),
        LBRROS2ControlMixin.node_controller_spawner(
            robot_name=robot_namespace, controller=LaunchConfiguration("ctrl")
        ),
    ]
    start_controllers = RegisterEventHandler(
        OnProcessStart(target_action=ros2_control_node, on_start=spawners)
    )

    gripper_driver = Node(
        package="wsg50_driver",
        executable="wsg50_gripper_driver_node",
        name="driver",
        namespace="wsg50",
        output="screen",
        parameters=[
            {
                "gripper_ip": LaunchConfiguration("gripper_ip"),
                "port": ParameterValue(LaunchConfiguration("gripper_port"), value_type=int),
            }
        ],
        remappings=[
            ("~/joint_states", ["/", robot_namespace, "/joint_states"]),
        ],
    )

    telemetry_bridge = Node(
        package="lbr_bringup",
        executable="admittance_telemetry_bridge",
        name="admittance_telemetry_bridge",
        namespace=robot_namespace,
        output="screen",
        parameters=[{"joint_prefix": robot_namespace}],
    )

    button_bridge = Node(
        package="button_bridge",
        executable="button_bridge",
        name="button_bridge",
        output="screen",
    )

    arguments = [
        DeclareLaunchArgument("robot_namespace", default_value="iiwa7"),
        DeclareLaunchArgument(
            "sys_cfg_pkg",
            default_value="lbr_demos_advanced_cpp",
            description="Package containing the active LBR hardware system configuration.",
        ),
        DeclareLaunchArgument(
            "sys_cfg",
            default_value="config/lbr_system_config.yaml",
            description="Path to the active LBR hardware system configuration.",
        ),
        LBRROS2ControlMixin.arg_init_jnt_pos_pkg(),
        LBRROS2ControlMixin.arg_init_jnt_pos(),
        LBRROS2ControlMixin.arg_ctrl_cfg_pkg(),
        LBRROS2ControlMixin.arg_ctrl_cfg(),
        DeclareLaunchArgument("ctrl", default_value="admittance_controller"),
        DeclareLaunchArgument("home_cfg_pkg", default_value="lbr_demos_advanced_cpp"),
        DeclareLaunchArgument("home_cfg", default_value="config/home_joint_positions.yaml"),
        DeclareLaunchArgument("gripper_ip", default_value="192.168.1.160"),
        DeclareLaunchArgument("gripper_port", default_value="1501"),
    ]

    return LaunchDescription(
        arguments
        + [
            robot_state_publisher,
            ros2_control_node,
            start_controllers,
            gripper_driver,
            telemetry_bridge,
            button_bridge,
        ]
    )

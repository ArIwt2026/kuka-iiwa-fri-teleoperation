"""Publish the calibrated Kuka hand-eye transform."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Values from calib/result_handeye.yaml:
    # calibrated_transform: iiwa7_link_ee -> d455_color_optical_frame
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="kuka_handeye_static_tf",
                output="screen",
                arguments=[
                    "--x",
                    "-0.021040187904981845",
                    "--y",
                    "0.040054241137663614",
                    "--z",
                    "0.05426520205256456",
                    "--qx",
                    "0.0069869452232783884",
                    "--qy",
                    "0.008155332711052721",
                    "--qz",
                    "0.3872469245808054",
                    "--qw",
                    "0.9219134951542509",
                    "--frame-id",
                    "iiwa7_link_ee",
                    "--child-frame-id",
                    "d455_color_optical_frame",
                ],
            )
        ]
    )

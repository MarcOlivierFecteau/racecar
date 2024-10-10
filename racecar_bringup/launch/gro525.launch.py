#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description() -> LaunchDescription:
    rosbridge_server_pkg_share_dir = get_package_share_directory('rosbridge_server')
    # web_video_server_pkg_share_dir = get_package_share_directory('web_video_server')
    # racecar_web_interface_pkg_share_dir = get_package_share_directory('racecar_web_interface')
    # racecar_bringup_pkg_share_dir = get_package_share_directory('racecar_bringup')

    port_argument = DeclareLaunchArgument(
        "port", default_value="9090", description="Port for rosbridge websocket"
    )

    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([rosbridge_server_pkg_share_dir, "/launch/rosbridge_websocket_launch.xml"]),
        launch_arguments = {"port": "9090"}.items()
    )

    web_video_server_node = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        parameters=[{"address": "10.42.0.1"}]
    )

    return LaunchDescription([
        port_argument,
        rosbridge_launch,
        web_video_server_node
    ])

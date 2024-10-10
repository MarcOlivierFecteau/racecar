from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directories
    rosbridge_server_pkg_share_dir = get_package_share_directory('rosbridge_server')
    # web_video_server_pkg_share_dir = get_package_share_directory('web_video_server')
    # racecar_web_interface_pkg_share_dir = get_package_share_directory('racecar_web_interface')
    
    port_argument = DeclareLaunchArgument(
        'port', default_value='9090',
        description='Port for rosbridge websocket'
    )
    

    # Launch rosbridge_websocket_launch.xml from rosbridge_server package
    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([rosbridge_server_pkg_share_dir, '/launch/rosbridge_websocket_launch.xml']),
        launch_arguments={'port': '9090'}.items()
    )

    #Launch web_video_server node from web_video_server package
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'address': '127.0.0.1'}]  # Set the address parameter to "10.42.0.1"
    )

    # Launch ros2_publisher_camera node from racecar_web_interface package
    ros2_publisher_camera_node = Node(
        package='racecar_web_interface',
        executable='ros2_publisher_camera',
        name='ros2_publisher_camera',
        output='screen'
    )

    # Launch all racecar teleoperation nodes (except joy_node; controller input will be sent from client)
    arduino_sensors_node = Node(
        package="racecar_bringup",
        executable="arduino_sensors",
        name="arduino_sensors",
        output="screen",
        remappings=[("/raw_odom", "prop_sensors")]
    )

    teleop_node = Node(
        package='racecar_teleop',
        executable='slash_teleop',
        name='teleop',
        output='screen',
    )

    controller_node = Node(
        package='racecar_autopilot',
        executable='slash_controller',
        name='controller',
        output='screen',
    )

    return LaunchDescription([
    	port_argument,
        rosbridge_launch,
        web_video_server_node,
        ros2_publisher_camera_node, 
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='pb2ros2',
                    executable='arduino_agent',
                    name='arduino',
                    output='screen',
                )
            ],
        ),
        arduino_sensors_node,
        teleop_node,
        controller_node
    ])

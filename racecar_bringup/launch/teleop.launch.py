from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from datetime import datetime

bag_name = f"app2_{datetime.now().strftime("%Y_%m_%d_%H%M_%S")}"

def generate_launch_description():
    return LaunchDescription([

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='racecar_serial_cpp',
                    executable='arduino_communication',
                    name='arduino',
                    output='screen',
                ),
            ],
        ),

        Node(
            package="racecar_bringup",
            executable="arduino_sensors",
            name="arduino_sensors",
            output="screen",
            remappings=[("/raw_odom", "prop_sensors")]
        ),

        Node(
            package='racecar_teleop',
            executable='slash_teleop',
            name='teleop',
            output='screen',
        ),

        Node(
            package='racecar_autopilot',
            executable='slash_controller',
            name='controller',
            output='screen',
        ),

        TimerAction(
            period=5.0,  # Adjust the delay duration as needed (in seconds)
            actions=[
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joy',
                    parameters=[{'deadzone': 0.05}],
                    arguments=['dev', '/dev/input/js0'],
                    output='screen',
                ),
            ]
        ),
    ])

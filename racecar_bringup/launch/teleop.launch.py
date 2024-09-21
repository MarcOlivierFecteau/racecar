from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
from datetime import datetime

bag_name = f"app2_{datetime.now().strftime("%Y_%m_%d_%H%M_%S")}"

def generate_launch_description():
    return LaunchDescription([
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='pb2ros2',
                    executable='arduino_agent',
                    name='arduino',
                    output='screen',
                ),
                ExecuteProcess(
                    cmd = [
                        "ros2", 
                        "bag", 
                        "record", 
                        "--output", 
                        f"/home/racecar/ros2_ws/bag_files/{bag_name}", 
                        "--max-bag-duration", 
                        "30", 
                        "--storage", 
                        "sqlite3", 
                        "--topics", 
                        ["/prop_sensors"]
                    ],
                )
            ],
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

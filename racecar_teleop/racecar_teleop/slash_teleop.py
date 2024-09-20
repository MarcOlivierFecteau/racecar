#!/usr/bin/env python3
import rclpy
from rclpy import Parameter
from rclpy.node import Node, Publisher, Subscription
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

PI: float = 3.141592


class Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


f710 = Namespace(
    axes=Namespace(LJH=0, LJV=1, LT=2, RJH=3, RJV=4, RT=5),
    buttons=Namespace(A=0, B=1, X=2, Y=3, LB=4, RB=5, BACK=6, START=7, HOME=8, LJP=9, RJP=10)
)


class Teleop(Node):
    def __init__(self):
        super().__init__("Teleop")
        self.max_velocity: Parameter = self.declare_parameter("max_vel", 4.0).value  # [m/s]
        self.max_voltage: Parameter = self.declare_parameter("max_volt", 8.0).value  # [V]
        self.max_steering_angle: Parameter = self.declare_parameter("max_angle", 40).value  # [deg]
        self.ps4: Parameter = self.declare_parameter("ps4", False).value

        self.cmd2rad: float = self.max_steering_angle * PI / 180
        self.warned_incompatible_controller: bool = False

        self.pub_cmd: Publisher = self.create_publisher(Twist, "ctl_ref", 1)
        self.sub_joy: Subscription = self.create_subscription(Joy, "joy", self.joy_callback, 1)

        self.cmd_msg = Twist(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))
        self.control_mode: int = -1

    def stop_all(self) -> None:
        self.control_mode = -1
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.linear.y = 0.0
        self.cmd_msg.angular.x = 0.0
        self.cmd_msg.angular.y = 0.0
        self.cmd_msg.angular.z = 0.0

    def joy_callback(self, msg: Twist) -> None:
        """
        | Mode | Position | Velocity | Steering | Other/Note |
        |------|----------|----------|----------|------------|
        | -1 |||| All-Stop ||
        | 0 | x | Closed | Open | prop <= 50% |
        | 1 | Open | Open | Open ||
        | 2 | Closed | x | Open ||
        | 3 | x | Closed | Closed ||
        | 4 | Closed | x | Closed ||
        | 5 | x | Closed | Closed | v = 1 m/s |
        | 6 |||| Reset encoder |
        | 7 |||| _Placeholder_ |
        """

        min_axes = 5 if self.ps4 else 4
        if len(msg.axes) < min_axes or len(msg.buttons) < 7:
            if not self.warned_incompatible_controller:
                self.get_logger().warning("[Slash Teleop] Controller is incompatible. Change controller to keep all functionalities.")
                self.warned_incompatible_controller = True
            return

        self.warned_incompatible_controller = False

        propulsion_user_input = (msg.axes[f710.axes.LT] - msg.axes[f710.axes.RT]) / 2
        steering_user_input = msg.axes[f710.axes.LJH]

        if not msg.buttons[f710.buttons.LB]:
            self.stop_all()
        else:
            if msg.buttons[f710.buttons.SELECT]:  # No ctl_ref msg published
                return
            if msg.buttons[f710.buttons.A]: # Fully open-loop (Boost mode)
                self.control_mode = 1
                self.cmd_msg.linear.x = propulsion_user_input * self.max_voltage    # [V]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
            elif msg.buttons[f710.buttons.B]:   # Closed-loop position, Open-loop steering
                self.control_mode = 2
                self.cmd_msg.linear.x = propulsion_user_input   # [m]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad
            elif msg.buttons[f710.buttons.X]:   # Closed-loop velocity, Closed-loop steering
                self.control_mode = 3
                self.cmd_msg.linear.x = propulsion_user_input * self.max_velocity   # [m/s]
                self.cmd_msg.angular.z = steering_user_input    # [m]
            elif msg.buttons[f710.buttons.Y]:   # Closed-loop position, Closed-loop steering
                self.control_mode = 4
                self.cmd_msg.linear.x = propulsion_user_input   # [m]
                self.cmd_msg.angular.z = steering_user_input    # [m]
            elif msg.buttons[f710.buttons.RB]:    # Closed-loop velocity with fixed 1 m/s ref, Closed-loop steering
                self.control_mode = 5
                self.cmd_msg.linear.x = 2.0 # [m/s]
                self.cmd_msg.angular.z = 0.0    # [m]
            elif msg.buttons[f710.buttons.BACK]:    # Reset Encoder
                self.control_mode = 6
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0
            elif msg.buttons[f710.buttons.HOME]:    # APP2 Labo1 mode 1
                self.control_mode = 0
                self.cmd_msg.linear.x = self.max_voltage / 8
                self.cmd_msg.angular.z = 0.0
            elif msg.buttons[f710.buttons.LJP]: # APP2 Labo1 mode 2
                self.control_mode = 0
                self.cmd_msg.linear.x = self.max_voltage / 4
                self.cmd_msg.angular.z = 0.0
            elif msg.buttons[f710.buttons.RJP]:
                self.control_mode = 0
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0
            else:   # Closed-loop velocity (<= 50% propulsion) Open-loop steering
                self.control_mode = 0
                self.cmd_msg.linear.x = propulsion_user_input * self.max_velocity   # [m/s]
                self.cmd_msg.angular.z = steering_user_input * self.cmd2rad

        self.cmd_msg.linear.z = self.control_mode
        self.pub_cmd.publish(self.cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

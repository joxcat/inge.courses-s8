#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Bool, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class Control (Node):
    def __init__(self):
        self.takeoff_state = False
        self.land_state = True
        self.flip_state = False

        # Node
        super().__init__("joy_control")

        # Publishers
        self.takeoff = self.create_publisher(Empty, "/takeoff", 10)
        self.land = self.create_publisher(Empty, "/land", 10)
        self.emergency = self.create_publisher(Empty, "/emergency", 10)
        self.control = self.create_publisher(Twist, "/control", 10)
        self.flip = self.create_publisher(String, "/flip", 10)
        self.toggle_qr = self.create_publisher(Bool, "/toggle_qr", 10)

        # Subscriber
        self.joystick_subscriber = self.create_subscription(Joy, "/joy", self.joystick_event, 10)


    def joystick_event(self, joy):

        if  joy.buttons[7] and self.land_state and not self.takeoff_state:
            self.get_logger().info("takeoff")
            self.takeoff.publish(Empty())
            self.takeoff_state = True
            self.land_state = False
            return

        if joy.buttons[6] and self.takeoff_state and not self.land_state:
            self.get_logger().info("land")
            self.land.publish(Empty())
            self.land_state = True
            self.takeoff_state = False
            return
        
        if joy.buttons[8]:
            self.get_logger().info("emergency")
            self.emergency.publish(Empty())
            return
        
        if joy.buttons[0] and not self.flip_state:
            self.flip_state = True
            flip = String()
            flip.data = 'f'
            self.flip.publish(flip)

        if not joy.buttons[0] and self.flip_state:
            self.flip_state = False

        if sum(joy.axes) >= 1:
            linear = Vector3()
            linear.x = -joy.axes[0]*50.0
            linear.y = joy.axes[1]*50.0
            linear.z = joy.axes[7]*50.0

            angular = Vector3()
            angular.x = 0.0
            angular.y = 0.0
            angular.z = -joy.axes[3]*50.0

            msg = Twist()
            msg.linear = linear
            msg.angular = angular

            self.control.publish(msg)
            return
        """
        else:
            linear = Vector3()
            angular = Vector3()

            msg = Twist()
            msg.linear = linear
            msg.angular = angular

            self.vel.publish(msg)
            """


def main():
    rclpy.init()
    joy_service = Control()
    rclpy.spin(joy_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
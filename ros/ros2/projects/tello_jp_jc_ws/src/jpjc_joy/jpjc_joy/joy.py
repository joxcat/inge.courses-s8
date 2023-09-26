#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Bool, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class Control (Node):
    def __init__(self):
        self.speed = 3
        self.takeoff_state = False
        self.land_state = True
        self.flip_state = False
        self.qr_button_state = False
        self.qr_mode = False

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

        self.get_logger().info("joy control started")

    def joystick_event(self, joy):
        if joy.axes[5] < -0.30:
            self.speed = 1
        elif joy.axes[5] >= -0.30 and joy.axes[5] <= 0.30:
            self.speed = 2
        elif joy.axes[5] > 0.30: 
            self.speed = 5

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

        if not self.qr_mode:    
            # self.get_logger().info("axes %s" % ", ".join(map(lambda a: str(a), joy.axes)))
            if joy.buttons[0] and not self.flip_state:
                self.flip_state = True
                flip = String()
                flip.data = 'f'
                self.flip.publish(flip)
                return

            if not joy.buttons[0] and self.flip_state:
                self.flip_state = False
                return

            if joy.buttons[1] and not self.qr_button_state:
                mode = Bool()
                mode.data = True
                self.toggle_qr.publish(mode)
                self.qr_button_state = True
                return

            if not joy.buttons[1] and self.qr_button_state:
                self.qr_mode = True
                self.qr_button_state = False
                return

            # By default without any input the LR and LF are at 1
            if sum(map(lambda x: abs(x), joy.axes[0:2] + joy.axes[3:5] + joy.axes[6:])) != 0:
                linear = Vector3()
                linear.x = -joy.axes[0] * 10.0 * self.speed
                linear.y = joy.axes[1] * 10.0 * self.speed
                linear.z = joy.axes[7] * 10.0 * self.speed

                angular = Vector3()
                angular.x = 0.0
                angular.y = 0.0
                angular.z = -joy.axes[3] * 20.0 * self.speed

                msg = Twist()
                msg.linear = linear
                msg.angular = angular

                self.control.publish(msg)
                return
            else:
                linear = Vector3()
                linear.x = 0.0
                linear.y = 0.0
                linear.z = 0.0

                angular = Vector3()
                angular.x = 0.0
                angular.y = 0.0
                angular.z = 0.0

                msg = Twist()
                msg.linear = linear
                msg.angular = angular

                self.control.publish(msg)
        else:
            if joy.buttons[1] and not self.qr_button_state:
                mode = Bool()
                mode.data = False
                self.toggle_qr.publish(mode)
                self.qr_button_state = True
                return

            if not joy.buttons[1] and self.qr_button_state:
                self.qr_mode = False
                self.qr_button_state = False
                return

def main():
    rclpy.init()
    joy_service = Control()
    rclpy.spin(joy_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
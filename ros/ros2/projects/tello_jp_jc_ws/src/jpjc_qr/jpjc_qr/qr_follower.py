#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Bool, String
from sensor_msgs.msg import Image

import sys
import time
import cv2
import imutils
import numpy as np
import sklearn
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3

class QrCodeFollower (Node):
    def __init__(self):
        # Node
        super().__init__("qr_follower")

        # Publishers
        self.control = self.create_publisher(Twist, "/control", 10)

        # Subscriber
        self.image_subscriber = self.create_subscription(Image, "/image_raw", self.video, 10)

    # Move according to QR code location
    def move(self, bbox):
        side_x_len = int(bbox[0][2][0] - bbox[0][0][0])
        side_z_len = int(bbox[0][2][1] - bbox[0][0][1])
        second_side_z_len = int(bbox[0][1][1] - bbox[0][3][1])
        diagonal = int(bbox[0][2][0] - bbox[0][0][1])

        x_center_offset = side_x_len / 2 + bbox[0][0][0] - 1024 / 2
        z_center_offset = side_z_len / 2 + bbox[0][0][1] - 576 / 2

        print(x_center_offset, z_center_offset)
        print(side_x_len)
        print("------")

        angular = Vector3()
        angular.x = 0.0
        angular.y = 0.0
        angular.z = 0.0

        linear = Vector3()
        linear.x = 0.0
        linear.y = 0.0
        linear.z = 0.0
        
        if x_center_offset > 30:
            linear.x = 25.0
        elif x_center_offset < -30:
            linear.x = -25.0

        if z_center_offset > 30:
            linear.z = -25.0
        elif z_center_offset < -30:
            linear.z = 25.0

        if diagonal > 60:
            linear.y = -25.0
        elif diagonal < 30:
            linear.y = 25.0

        if side_z_len > second_side_z_len:
            angular.z = -5.0
        else:
            angular.z = 5.0

        msg = Twist()
        msg.angular = angular
        msg.linear = linear

        self.control.publish(msg)

    def video(self, image):
        try:
            frame = CvBridge().imgmsg_to_cv2(image, desired_encoding='passthrough')

            key = cv2.waitKey(1) & 0xFF
        
            qr_decoder = cv2.QRCodeDetector()
        
            # Detect and decode the qrcode
            data, bbox, rectified_image = qr_decoder.detectAndDecode(frame)

            if len(data) > 0:
                self.move(bbox)
            
        except Exception as e:
            self.get_logger().info(e)

def main():
    rclpy.init()
    qr_code_follower = QrCodeFollower()
    rclpy.spin(qr_code_follower)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
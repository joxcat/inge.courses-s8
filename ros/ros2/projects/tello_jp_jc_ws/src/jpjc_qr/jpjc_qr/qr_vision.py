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
from cv_bridge import CvBridge

class QrCodeVision (Node):
    def __init__(self):
        # Node
        super().__init__("qr_vision")

        # Publishers
        #self.control = self.create_publisher(Twist, "/control", 10)

        # Subscriber
        self.image_subscriber = self.create_subscription(Image, "/image_raw", self.video, 10)

    # Display barcode and QR code location
    def display(self, img, bbox):
        n = len(bbox)

        pt1 = int(bbox[0][0][0]), int(bbox[0][0][1])    # angle en haut à gauche
        pt2 = int(bbox[0][2][0]), int(bbox[0][2][1])    # angle en bas à droite
        
        color = (255, 0, 0)

        cv2.rectangle(img, pt1, pt2, color)

        # Display results
        cv2.imshow("Results", img)

    def video(self, image):
        frame = CvBridge().imgmsg_to_cv2(image, desired_encoding='passthrough')

        cv2.imshow("frame", frame)
        key = cv2.waitKey(1) & 0xFF
                
        #example code from google TODO: link
        qr_decoder = cv2.QRCodeDetector()
    
        # Detect and decode the qrcode
        data, bbox, rectified_image = qr_decoder.detectAndDecode(frame)
        if len(data) > 0:
            # print("Decoded Data : {}".format(data))
            self.display(frame, bbox)
            rectified_image = np.uint8(rectified_image);
            cv2.imshow("Rectified QRCode", rectified_image);
        else:
            # print("QR Code not detected")
            cv2.imshow("Results", frame)

def main():
    rclpy.init()
    qr_code_vision = QrCodeVision()
    rclpy.spin(qr_code_vision)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from math import isnan

distance_pipe = None

def send_distance(distance):
    try:
        distance_pipe.publish(distance)
    except rospy.ROSInterruptException:
        pass

def on_scan(data):
    min = 0
    for distance in data.ranges:
        if (not(isnan(distance)) and (min == 0 or distance < min)):
            min = distance

    rospy.loginfo("WOW GET BACK YOU ARE TOO CLOSE " + str(min) + " " + rospy.get_caller_id())
    send_distance(min)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, on_scan)
    distance_pipe = rospy.Publisher('/min_dist', Float32, queue_size=10)
    rospy.spin()


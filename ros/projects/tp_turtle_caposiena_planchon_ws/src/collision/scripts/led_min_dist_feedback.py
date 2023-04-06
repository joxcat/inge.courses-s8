#!/usr/bin/env python3

import rospy
from kobuki_msgs.msg import Led
from std_msgs.msg import Float32
from math import isnan, pow

class SoundMinDistFeedback:
    def __init__(self):
        self.max_rate = 30.0
        self.min_rate = 1.0
        self.max_distance = 10.0

        # N
        rospy.init_node('sound_min_dist_feedback', anonymous=True)
        # P
        self.lighter = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)
        # S
        self.rate = rospy.Rate(self.min_rate)
        
        rospy.Subscriber('/min_dist', Float32, self.on_min_dist)

        self.beeper()

    def toggle_led(self, color):
        try:
            self.lighter.publish(Led(value=color))
        except rospy.ROSInterruptException:
            pass

    def on_min_dist(self, raw_distance):
        distance = float(raw_distance.data)
        computed_rate = self.max_rate - (distance / self.max_distance * (self.max_rate - self.min_rate))
        rospy.loginfo("WOW YOU ARE COMING " + str(computed_rate) + rospy.get_caller_id())
        self.rate = rospy.Rate(computed_rate / 2)

    def beeper(self):
        while not rospy.is_shutdown():
            self.toggle_led(3)
            self.rate.sleep()
            self.toggle_led(0)
            self.rate.sleep()


if __name__ == '__main__':
    c = SoundMinDistFeedback()
    rospy.spin()

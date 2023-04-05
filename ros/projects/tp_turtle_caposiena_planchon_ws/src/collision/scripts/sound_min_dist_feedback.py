#!/usr/bin/env python3

import rospy
from kobuki_msgs.msg import Sound
from std_msgs.msg import Float32
from math import isnan

class SoundMinDistFeedback:
    def __init__(self):
        self.max_rate = 5.0
        self.min_rate = 0.5
        self.max_distance = 10.0

        # N
        rospy.init_node('sound_min_dist_feedback', anonymous=True)
        # P
        self.bipper = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
        # S
        self.rate = rospy.Rate(self.min_rate)
        
        rospy.Subscriber('/min_dist', Float32, self.on_min_dist)

        self.beeper()

    def beep(self, sound):
        try:
            self.bipper.publish(Sound(value=sound))
        except rospy.ROSInterruptException:
            pass

    def on_min_dist(self, raw_distance):
        distance = float(raw_distance.data)
        computed_rate = self.max_rate - (distance / self.max_distance * (self.max_rate - self.min_rate))
        rospy.loginfo("WOW YOU ARE COMING " + str(computed_rate) + rospy.get_caller_id())
        self.rate = rospy.Rate(computed_rate)

    def beeper(self):
        while not rospy.is_shutdown():
            self.beep(3)
            self.rate.sleep()

if __name__ == '__main__':
    c = SoundMinDistFeedback()
    rospy.spin()

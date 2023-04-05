#!/usr/bin/env python3

import rospy
from kobuki_msgs.msg import BumperEvent, Sound

bipper = None

def beep(sound):
    try:
        bipper.publish(Sound(value=sound))
    except rospy.ROSInterruptException:
        pass


def on_bumper_event(data):
    rospy.loginfo("HELP SOMEONE BUMBED MEE " + rospy.get_caller_id())

    if (data.bumper == 0):
        beep(1)
    elif (data.bumper == 1):
        beep(2)
    elif (data.bumper == 2):
        beep(4)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, on_bumper_event)
    bipper = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
    rospy.spin()

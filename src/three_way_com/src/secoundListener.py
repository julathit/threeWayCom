#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s", data.data)
    if data.data == "p":
        print("shutdown the node")
        rospy.signal_shutdown("Received 's', shutting down subscriber.")

def listener():
    rospy.init_node('listener2', anonymous=True)
    rospy.Subscriber('chatter2', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

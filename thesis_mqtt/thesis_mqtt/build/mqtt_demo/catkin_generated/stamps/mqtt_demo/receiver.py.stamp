#! usr/bin/env python

import rospy
from std_msgs.msg import String

def rob_ball_position_callback(message):
    rospy.loginfo("Received message from Mqtt: " + message)

def listener():
    rospy.init_node('pos_receiver', anonymous = True)
    rospy.Subscriber("pos_received", String, rob_ball_position_callback)

    rospy.spin()

if __name__ == 'main':
    rospy.loginfo("Hello")
    print("Hello")
    #listener()
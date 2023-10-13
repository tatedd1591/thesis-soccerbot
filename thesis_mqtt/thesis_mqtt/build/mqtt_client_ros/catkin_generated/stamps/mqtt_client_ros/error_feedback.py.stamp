#! usr/bin/env python

import rospy
from std_msgs.msg import String
import math

def error_feedback():
    feedback = rospy.Publisher('/error_pub/primitive', String, queue_size=10)
    rospy.init_node('error_publisher', anonymous = True)
    rate = rospy.Rate(100)
    i = 0
    j = 0
    while not rospy.is_shutdown():
        error = "Bot:" + str(round(math.cos(i*2), 2)) + ":" + str(0.5) + ":" + str(j) 
        # rospy.loginfo(error)
        feedback.publish(error)
        rate.sleep()
        i=i+3.14/3
        j += 1

if __name__=='__main__':
    error_feedback()


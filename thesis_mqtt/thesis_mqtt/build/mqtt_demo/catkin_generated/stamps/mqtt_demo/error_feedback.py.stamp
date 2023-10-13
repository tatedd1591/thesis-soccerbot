#! usr/bin/env python

import rospy
from std_msgs.msg import String

def error_feedback():
    feedback = rospy.Publisher('error_published', String, queue_size=10)
    rospy.init_node('error_publisher', anonymous = True)
    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown():
        error = "error_demo %s" % i
        rospy.loginfo(error)
        feedback.publish(error)
        rate.sleep()
        i=i+1

if __name__=='main':
    error_feedback()


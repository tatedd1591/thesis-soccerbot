#! usr/bin/env python

import rospy
from std_msgs.msg import String

def error_feedback():
    feedback = rospy.Publisher('/pos_pub/primitive', String, queue_size=10)
    rospy.init_node('error_publisher', anonymous = True)
    rate = rospy.Rate(100)
    i = 0
    j=0
    while not rospy.is_shutdown():
        error = "Host:" + str(i) + ":" +  str(i*(-0.5)) + ":" + str(j)
        rospy.loginfo(error)
        feedback.publish(error)
        rate.sleep()
        i=(i+1)*(-1)
        j+=1

if __name__=='__main__':
    error_feedback()


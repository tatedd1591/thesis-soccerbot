#! usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
import string

def rob_ball_position_callback(message):
	if (message.data.find('Angle ') != -1):
		angle = message.data.split('Angle ')
		angle_publisher.publish(int(angle[1]))
	if (message.data.find('Distance ') != -1):
		dis = message.data.split('Distance ')
		dis_publisher.publish(float(dis[1]))
	if (message.data.find('Mode ') != -1):
		mode = message.data.split('Mode ')
		if mode[1] == "11":
			mode_publisher.publish(11)
		elif mode[1] == "12":
			mode_publisher.publish(12)
		elif mode[1] == "13":
			mode_publisher.publish(13)
		elif mode[1] == "21":
			mode_publisher.publish(21)
		elif mode[1] == "23":
			mode_publisher.publish(23)
		elif mode[1] == "31":
			mode_publisher.publish(31)
		elif mode[1] == "32":
			mode_publisher.publish(32)
		elif mode[1] == "33":
			mode_publisher.publish(33)
		elif mode[1] == "66":
			mode_publisher.publish(66)
		elif mode[1] == "77":
			mode_publisher.publish(77)
		elif mode[1] == "S":
			mode_publisher.publish(55)
		elif mode[1] == "C":
			mode_publisher.publish(44)
	

def listener():
	rospy.init_node('pos_receiver', anonymous = True)
	rospy.Subscriber("pos_rec/primitive", String, rob_ball_position_callback)
	rospy.spin()

if __name__ == '__main__':
	mode_publisher = rospy.Publisher('mode_topic', UInt8, queue_size = 10)
	dis_publisher = rospy.Publisher('dis_topic', Float32, queue_size = 10)
	angle_publisher = rospy.Publisher('angle_topic', UInt8, queue_size = 10)
#	rospy.init_node('test_node', anonymous = True)
	listener()



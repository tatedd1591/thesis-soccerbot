#! usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
import string
from std_msgs.msg import Float32MultiArray

def rob_ball_position_callback(message):
	if (message.data.find('Host:') != -1):
		
		data = message.data.split(':')
		data.remove('Host')
		pos_msg = Float32MultiArray()
		data=[float(data[i]) for i in range(len(data))]
		pos_msg.data=data		
		data_publisher.publish(pos_msg)
		# print(len(data))
		# print(message.data)

def listener():
	rospy.init_node('pos_receiver', anonymous = True)
	rospy.Subscriber("/pos_rec/primitive", String, rob_ball_position_callback)
	rospy.spin()

if __name__ == '__main__':
	data_publisher = rospy.Publisher('data_topic', Float32MultiArray, queue_size = 10)
	# rospy.init_node('test_node', anonymous = True)
	listener()

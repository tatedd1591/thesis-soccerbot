#!/usr/bin/env python3

import rospy
from serial_stm32.msg import uartUSB
from std_msgs.msg import String
import serial

def vel_publish():
    serial_port = serial.Serial(port = "/dev/ttyUSB0", 
                                baudrate = 115200, 
                                bytesize = 8, 
                                timeout = 0)
    publish = rospy.Publisher('vel_pub_topic', String, queue_size=10)
    rospy.init_node('vel_pub', anonymous = True)
    rate = rospy.Rate(1)
    i = 0
    while not rospy.is_shutdown():
        data = uartUSB()
        data.dir1 = 0x61
        rospy.loginfo(data.dir1)
        publish.publish(data.dir1)
        rate.sleep()
        i=i+1

if __name__=='__main__':
    vel_publish()

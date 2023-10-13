#include <ros/ros.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "wiringPiSPI.h"
#include "wiringPiI2C.h"
#include <stdio.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <turtlesim/Pose.h>
#include <vector>
using namespace std;
void datasubcallback(const std_msgs::UInt8::ConstPtr& msg);
// GIA TRI TUYEN (ROS TO MQTT)
std_msgs::String vel_msg;
ros::Publisher vel1_pub;
std::stringstream ss;
//void convert(char msg->data);
const double pi = 3.1415926;
int fd1 = serialOpen("/dev/ttyAMA0", 115200);
int count_data=0;
char error1_sub;
char get_data=0;
std::vector<string> vel1_sub;


void datasubcallback(const std_msgs::UInt8::ConstPtr& msg)
{              
}
int main(int argc, char **argv){
    ros::init(argc, argv, "test_wiringpi_spi1");
    ros::NodeHandle node;

    if (fd1 == -1){
        ROS_INFO("Failed");
        return -1;
    }      
    ROS_INFO("Success");
    
    // fflush(stdout);
    ros::Publisher vel1_pub = node.advertise<std_msgs::String>("/error_pub/primitive", 1000);
    ros::Subscriber error1_sub = node.subscribe("/signal_pub", 1000, datasubcallback);
    while (ros::ok())
    {
        // printf("sadasdsa %d\n",g);
            //a=0;
            count_data=0;
            vel1_sub="Bot:";
            // vel1_sub[0]="1123:1123:1123";
            // if (g=0)
            // {
            do
            {           
            if (count_data%4==0 && count_data !=0)
            {
            vel1_sub=vel1_sub+":";
            }
            get_data=serialGetchar(fd1);
            printf("Data want to transmit %c\n",get_data);
		    vel1_sub = vel1_sub+get_data;
		    fflush (stdout);
            count_data++;
	        }
            while(serialDataAvail(fd1));
            vel_msg.data = vel1_sub.c_str(); 
            vel1_pub.publish(vel_msg);
            // }
            
            
            // if (g == 1)
            // {
            //     for (int i=0;i<a;i++)
            //     {
            //     vel_msg.data = vel1_sub[i].c_str(); 
            //     // printf("Vel bot: %s\n",vel1_sub[a].c_str());
            //     vel1_pub.publish(vel_msg);
            //     printf("%s\n",vel1_sub[i].c_str());
            //     ros::Duration(0.04).sleep(); 
            //     }
            // }
            //ros::Duration(0.01).sleep();
        
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

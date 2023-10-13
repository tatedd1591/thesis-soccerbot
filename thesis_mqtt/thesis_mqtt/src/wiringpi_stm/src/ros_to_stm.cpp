#include <ros/ros.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "wiringPiSPI.h"
#include "wiringPiI2C.h"
#include <stdio.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

void velsubcallback(const std_msgs::UInt8::ConstPtr& msg);
void dissubcallback(const std_msgs::Float32::ConstPtr& msg);
void anglesubcallback(const std_msgs::UInt8::ConstPtr& msg);
void vel_calculate(double *v1, double *v2, double *v3);
void vel_convert(char* vel_pub);

const double pi = 3.1415926;
int fd = serialOpen("/dev/ttyAMA0", 115200);
float time_delay = 0;
double v_x, v_y, omega;
double v1 = 0, v2 = 0, v3 = 0;
char vel_pub[13];
int vel_linear = 30;

void vel_calculate(double *v1, double *v2, double *v3)
{
    *v1 = -v_y + 10* omega;
    *v2 = v_x*cos(pi/6) + v_y*cos(pi/3) + 10 * omega;
    *v3 = -v_x*cos(pi/6) + v_y*cos(pi/3) + 10 * omega;
}

void vel_convert(char* vel_pub){
    char dir1, dir2, dir3;
    if (v1 < 0) dir1 = '1'; else dir1 = '0';
    if (v2 < 0) dir2 = '1'; else dir2 = '0'; 
    if (v3 < 0) dir3 = '1'; else dir3 = '0';

    v1 = abs(v1)*60/(5.8*pi);
    v2 = abs(v2)*60/(5.8*pi);
    v3 = abs(v3)*60/(5.8*pi);
    
    printf("v1: %.2f, v2: %.2f, v3: %0.f\n", v1, v2, v3);

    sprintf(vel_pub, "%c%d%c%d%c%d", dir1, int(round(v1+100)), dir2, int(round(v2+100)), dir3, int(round(v3+100)));
    printf("vel_pub: %s\n", vel_pub);
}

void anglesubcallback(const std_msgs::UInt8::ConstPtr& msg)
{
    printf("Desesired angle: %d\n", msg->data);
    float del_angle = 20.0/180*msg->data;
    time_delay = (float(msg->data-del_angle)/360)*10*2*pi/(8*5.8*pi*10/60);
    //time_delay = 0.5*100/vel_linear;
    printf("Time to delay: %f\n", time_delay);
}

void dissubcallback(const std_msgs::Float32::ConstPtr& msg)
{
    printf("Desesired distance: %f\n", msg->data);
    float del_distance = 6.0/100*msg->data;
    time_delay = float((msg->data)-del_distance)*100/vel_linear;
    //time_delay = 0.5*100/vel_linear;
    printf("Time to delay: %f\n", time_delay);
}

void velsubcallback(const std_msgs::UInt8::ConstPtr& msg)
{
    printf("Mode: %d: \n",msg->data);
    switch (msg->data){
    case 11:
        v_x = -vel_linear*cos(pi/4);
        v_y = vel_linear*cos(pi/4);
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        printf("v1: %.2f, v2: %.2f, v3: %0.f\n", v1, v2, v3);
        vel_convert(vel_pub);
        //vel_pub = "110000000100";
        printf( "Car is moving up left\n");
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep(); 
        serialPuts(fd, "000000000000");
        fflush(stdout);
        // fflush(stdout);
        break;
    case 12:
        //vel_pub = "110000000100";
        v_x = 0;
        v_y = vel_linear;
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        printf("v_x: %.2f, v_y: %.2f, v1: %.2f, v2: %.2f, v3: %0.f\n", v_x, v_y, v1, v2, v3);
        vel_convert(vel_pub);

        printf( "Car is moving up\n");
        serialPuts(fd, vel_pub);
        // fflush(stdout);
        ros::Duration(time_delay).sleep();  
        serialPuts(fd, "000000000000");
        fflush(stdout);
        break;
    case 13:
        v_x = vel_linear*cos(pi/4);
        v_y = vel_linear*cos(pi/4);
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        printf("v1: %.2f, v2: %.2f, v3: %0.f\n", v1, v2, v3);
        vel_convert(vel_pub);

        printf( "Car is moving up right\n");
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();  
        serialPuts(fd, "000000000000");
        fflush(stdout);
        break;
    case 21:
        v_x = -vel_linear;
        v_y = 0;
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        printf("v1: %.2f, v2: %.2f, v3: %0.f\n", v1, v2, v3);
        vel_convert(vel_pub);

        printf( "Car is moving left\n");
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();  
        serialPuts(fd, "000000000000");
        fflush(stdout);
        break;
    case 23:
        v_x = vel_linear;
        v_y = 0;
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        printf("v1: %.2f, v2: %.2f, v3: %0.f\n", v1, v2, v3);
        vel_convert(vel_pub);
        
        printf( "Car is moving right\n");
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();  
        serialPuts(fd, "000000000000");
        fflush(stdout);
        break;
    case 31:
        v_x = -vel_linear*cos(pi/4);
        v_y = -vel_linear*cos(pi/4);
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        printf("v1: %.2f, v2: %.2f, v3: %0.f\n", v1, v2, v3);
        vel_convert(vel_pub);

        printf( "Car is moving down left\n");
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();  
        serialPuts(fd, "000000000000");
        fflush(stdout);
        break;
    case 32:
        v_x = 0;
        v_y = -vel_linear;
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        printf("v1: %.2f, v2: %.2f, v3: %0.f\n", v1, v2, v3);
        vel_convert(vel_pub);

        printf( "Car is moving down\n");
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();  
        serialPuts(fd, "000000000000");
        fflush(stdout);
        break;
    case 33:
        v_x = vel_linear*cos(pi/4);
        v_y = -vel_linear*cos(pi/4);
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        printf("v1: %.2f, v2: %.2f, v3: %0.f\n", v1, v2, v3);
        vel_convert(vel_pub);

        printf( "Car is moving down right\n");
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();  
        serialPuts(fd, "000000000000");
        fflush(stdout);
        break;
    case 66:
        v_x = 0;
        v_y = 0;
        omega = -8*5.8*pi/60;
        vel_calculate(&v1, &v2, &v3);
        printf("v1: %.4f, v2: %.4f, v3: %0.4f\n", v1, v2, v3);
        vel_convert(vel_pub);

        printf( "Car is rotating clockwise\n");
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();  
        serialPuts(fd, "000000000000");
        fflush(stdout);
        break;
    case 77:
        v_x = 0;
        v_y = 0;
        omega = 8*5.8*pi/60;
        vel_calculate(&v1, &v2, &v3);
        printf("v1: %.4f, v2: %.4f, v3: %0.4f\n", v1, v2, v3);
        vel_convert(vel_pub);

        printf( "Car is rotating couter clockwise\n");
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();  
        serialPuts(fd, "000000000000");
        fflush(stdout);
        break;
    case 44:
        v_x = 0;
        v_y = 25;
        omega = 3*5.8*pi/60;
        vel_calculate(&v1, &v2, &v3);
        printf("v1: %.4f, v2: %.4f, v3: %0.4f\n", v1, v2, v3);
        vel_convert(vel_pub);
        printf( "Car is rotating clockwise\n");
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(6.0833).sleep();  
        serialPuts(fd, "000000000000");
        fflush(stdout);
        break;
    case 55:
        v_x = 0;
        v_y = vel_linear;
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        vel_convert(vel_pub);
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();  
        v_x = vel_linear;
        v_y = 0;
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        vel_convert(vel_pub);
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();
        v_x = 0;
        v_y = -vel_linear;
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        vel_convert(vel_pub);
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();
        v_x = -vel_linear;
        v_y = 0;
        omega = 0;
        vel_calculate(&v1, &v2, &v3);
        vel_convert(vel_pub);
        serialPuts(fd, vel_pub);
        fflush(stdout);
        ros::Duration(time_delay).sleep();  
        serialPuts(fd, "000000000000");
        fflush(stdout);
        break;
    default:
        break;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "test_wiringpi_spi");
    ros::NodeHandle node;
    
    if (fd == -1){
        ROS_INFO("Failed");
        return -1;
    }
    ROS_INFO("Success");
    
    // fflush(stdout);
    ros::Subscriber mode_sub = node.subscribe("mode_topic", 1000, velsubcallback);
    ros::Subscriber dis_sub = node.subscribe("dis_topic", 1000, dissubcallback);
    ros::Subscriber angle_sub = node.subscribe("angle_topic", 1000, anglesubcallback);
    ros::spin();
}

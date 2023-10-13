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

// GIA TRI TUYEN (ROS TO MQTT)
std_msgs::String error_msg;
ros::Publisher error_pub;
std_msgs::String signal_msg;
ros::Publisher signal_pub;
void modesubcallback(const std_msgs::UInt8::ConstPtr& msg);
void datasubcallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void vel_calculate(double *v1, double *v2, double *v3);
void vel_convert(char* vel_pub);
void path(float x,float y,float theta,float x1, float y1);
void calculate_error(void);
void velbot(void);
void position(void);
void receive_value(void);

//void convert(char msg->data);
const double pi = 3.1415926;
int fd = serialOpen("/dev/ttyAMA0", 115200);
double v1 = 0, v2 = 0, v3 = 0;
int dir0=2;
char vel_pub[14];
char vel_sub[13];
int a=0;
// MAPPING
float theta0;                   // Chon mat sut bong
float theta12,theta23;          // Goc lech doan 12 va 23 so voi truc X
float alpha12,alpha23;          // Goc lech doan 12,23 so voi xB

//VI TRI
// Diem bat dau-Start point (1)
float x_start,y_start,theta_start;
// Diem lay da - Speed point (2)
float xt,yt;
// Cap nhat vi tri
float xbot,ybot,thetabot;       // Vi tri robot
float xball,yball;              // Vi tri bong
float xB,yB;
//GIA TRI TRUYEN (ROS TO STM)
double vf=0,vn=0,v=0;       // Van toc tiep, phap, goc va van toc dai trong toa do robot

// SAI SO
float xerror,yerror;            // Vi tri can tinh sai so
// Sai so
float e1 = 0;                   // Sai so khoang cach
float e2 = 0;                   // Sai so goc vector van toc voi quy dao
float e3 = 0;                   // Sai so goc mat sut robot mong muon
// He so e3
float kpE3 = 3 ;
float kdE3 = 0 ;
float kiE3 = 0 ;
float pre_e3=0;
float ipartE3=0;
// He so e1
float kpE1 = 2.5;
float kdE1 = 0 ;
float kiE1 = 0 ;
float pre_e1=0;
float ipartE1=0;

// DAI LUONG MONG MUON
float vR=500;                    // Toc do tham chieu can dat.
float wR;                       // Toc do goc                             
float dt = 0.04 ;               // Thoi gian lay mau 
float thetamm=22*pi/180;
// THONG SO CO BAN CUA ROBOT
uint8_t r = 58/2  ;             // Ban kinh banh xe.
uint8_t R = 90    ;             // Ban kinh khung xe
uint8_t dw = 13   ;             // 1 nua do day banh xe
uint8_t dd=0;
// OUTPUT 
float w1 = 0, w2 = 0, w3 = 0,w;   // Van toc goc 3 banh xe

// XET CASE
uint8_t step = 0;               // TH di chuyen
uint8_t path_num = 0;           // Chon mapping
uint8_t count=0;
uint8_t countd=0;
void path(float x,float y,float theta,float x1, float y1)
{
    // Vi tri bat dau
    xB=x1;
    yB=y1;
    theta_start=thetabot;
    x_start=xbot;
    y_start=ybot;
    xt = 450   ;                               // Toa do x
    yt = -300;                                           // Toa do y
    // Vi tri lay da
    // Start -> diem lay da (12)
    theta12 = atan2(yt-y_start,xt-x_start);                 // Goc lech doan 12 so voi truc x
    alpha12 = -theta_start + theta12;                       // Goc lech vector v so voi 12
    // Diem lay da -> bong (23)
    theta23 = thetamm;                         // Goc lech doan 23 so voi truc x
    alpha23 = -theta_start + theta23;                       // Goc lech vector v so voi 23
    // Chon mat sut bong
    if      (alpha23>pi/3)      theta0 = 2*pi/3;            // Mat 2 ( Doi dien dong co 2 )
    else if (alpha23<-pi/3)     theta0 = -2*pi/3;           // Mat 3 ( Doi dien dong co 3 )
    else                        theta0 = 0;                 // Mat 1 ( doi dien dong co 1 )
    // Xet cac diem can den cho toi diem lay da
    xerror = xt;    yerror = yt;
    e1 = sqrt((xerror-x_start)*(xerror-x_start)+(yerror-y_start)*(yerror-y_start));
    e2 = -thetabot+atan2(yerror-y_start,xerror-x_start);                                                                        
    e3 = -thetabot-theta0+thetamm;
}

void calculate_error(void)
{
    pre_e3=e3;
    e1 = sqrt((xerror-xbot)*(xerror-xbot)+(yerror-ybot)*(yerror-ybot));
    e2 = -thetabot+atan2(yerror-ybot,xerror-xbot);                                                                        
    e3 = -thetabot-theta0+thetamm;
    printf("e1: %.2f, e2 %.2f, e3: %.2f \n", e1, e2*180/pi, e3*180/pi);
    //printf("dadasda: %s\n", error_msg.data.c_str());
}
void velbot(void)
{   
    // ipartE1=ipartE1+(e1+pre_e1)*dt;
    vR = kpE1*e1+kiE1*ipartE1+kdE1*(e1-pre_e1)/dt;
    if (e1>200)         vR=500;
    if (step==1) vR=0;
    // ipartE3=ipartE3+(e3+pre_e3)*dt;
    w = 3*e3;
    if (w>pi/6)       w = pi/6;
    else if (w<-pi/6) w = -pi/6;
    if (step==1) w=0;
    vf=vR*cos(e2);
    vn=vR*sin(e2);
    std::stringstream ss;
    if (countd<=10) 
    {ss << "Bot:"<< float(e1) << ":" << float(vR) << ":" << float(w) << ":" << float(e3);
    error_msg.data = ss.str();
    }
    if (dd==2) 
    {ss << "1";
    signal_msg.data = ss.str();
    }
    printf("w: %f e1: %f\n", w,e1);
}

void position(void)
{
    calculate_error();                         // Chi tinh sai so trung step 0
    switch (step)
    {
        case 0:                                             // Di chuyen den diem lay da
        {
            dd=1;
            // calculate_error();
            if ((abs(e1)>10 || abs(e3) > pi/360))       // Chua dat den sai so chap nhan
            {
                velbot();
                break;
            }
            else                                        // Dat den sai so chap nhan
            {
                vf=0;
                vn=0;
                w=0;
                step=1; 
            }                                              
        }
        case 1:                                                 // Ket thuc chuong trinh
        { 
            countd++;      
            vR=0;
            vf=0;
            vn=0;
            w=0;
            velbot();
            if (countd>10)  dd=2;
            break;
        }
    }
}

void vel_calculate(double *v1, double *v2, double *v3)
{
    *v1 = -vn+R*w;
    *v2 = vf*cos(pi/6)  + vn*cos(pi/3) + R*w;
    *v3 = -vf*cos(pi/6) + vn*cos(pi/3) + R*w;
    // printf("v1: %.2f, v2: %.2f, v3: %.2f\n", *v1, *v2, *v3);
}
void vel_convert(char* vel_pub){
    char dir1, dir2, dir3;
    count++;
    printf("w1: %.2f, w2: %.2f, w3: %.2f\n", v1, v2, v3);
    if (v1 < 0) dir1 = '1'; else dir1 = '0';
    if (v2 < 0) dir2 = '1'; else dir2 = '0'; 
    if (v3 < 0) dir3 = '1'; else dir3 = '0';

    v1 = (abs(v1)*30)/(r*pi);
    v2 = (abs(v2)*30)/(r*pi);
    v3 = (abs(v3)*30)/(r*pi);
      
    sprintf(vel_pub, "%c%d%c%d%c%d%d", dir1, int(round(v1+100)), dir2, int(round(v2+100)), dir3, int(round(v3+100)),dir0);
    // if (count==5)
    // {
        
        printf("vel_pub: %s\n", vel_pub);
        // count=0;
    // } 
}

void modesubcallback(const std_msgs::UInt8::ConstPtr& msg)
{
}

void datasubcallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // Doc vi tri robot
    xbot        = 1200-float(msg->data[0]);
    ybot        = float(msg->data[1])-600;
    thetabot    = float(msg->data[2])*pi/180;
    xball       = 1200-float(msg->data[3]);
    yball       = float(msg->data[4])-600;
    // Tao quy dao
    if (path_num==0)
    {
        path(xbot,ybot,thetabot,xball,yball);
        path_num=1;
    }
    // Kiem tra vi tri robot va diem sut bong
    printf("bot: %f %f %f %f %f \n", 1200-xbot,ybot+600,thetabot*180/pi,1200-xball,yball+600);
    printf("kicking point: %f %f %d %f %f\n", xt,yt,step,theta0*180/pi,alpha12*180/pi);
    // Xet TH robot di chuyen
   position(); 
    signal_msg.data = 1;
    //Test dong co
    // w=-pi/2;
    // vf=0;
    // vn=0;
    
    // Tinh toan van toc dong co 
    vel_calculate(&v1, &v2, &v3);
    vel_convert(vel_pub);
    // Gui du lieu gui ve STM
    serialPuts(fd, vel_pub);
    fflush(stdout);
    // receive
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
    //ros::Publisher error_pub = node.advertise<std_msgs::String>("/error_pub/primitive", 1000);
    //ros::Publisher signal_pub = node.advertise<std_msgs::UInt8>("/signal_pub", 1000);
    ros::Subscriber data_sub = node.subscribe("data_topic", 1000, datasubcallback);
    ros::Subscriber mode_sub = node.subscribe("mode_topic", 1000, modesubcallback);
    while (ros::ok())
    {
        // if (dd==1)
        // {
        // error_pub.publish(error_msg);
        // ros::Duration(0.04).sleep();       
        // }
        // if (dd==2)
        // {
        //     signal_pub.publish(signal_msg);
        //     dd++;
        // }
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

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

void modesubcallback(const std_msgs::UInt8::ConstPtr& msg);
void datasubcallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void vel_calculate(double *v1, double *v2, double *v3);
void vel_convert(char* vel_pub);
void path(float x,float y,float theta,float x1, float y1);
void giaodiem(float xM,float yM, float thetaM);
void calculate_error(void);
void position(void);
void receive_value(void);

//void convert(char msg->data);
const double pi = 3.1415926;
int fd = serialOpen("/dev/ttyAMA0", 115200);
float time_delay = 0;
double v1 = 0, v2 = 0, v3 = 0;
char vel_pub[13];
int vel_linear = 30;

float xbot,ybot,thetabot,xball,yball;   //vi tri robot va vi tri bong

// mapping
float a1,b1;            // Duong 12 y=a1*x+b1
float a2,b2;            // Duong 23 y=a2*x+b2
float a3,b3;            // Duong sai so e2 : y=a3*x+b3
float theta0;           // Chon mat sut bong
float theta12,theta23;  // Goc lech doan 12 va 23 so voi truc X
float alpha12,alpha23;  // Goc lech doan 12,23 so voi xB
float rho12,rho23;      // Khoang cach doan 12,23

//gia tri truyen
double vf=0,vn=0,w=0,v=0;   // Van toc tiep, phap, goc trong toa do robot

//diem bat dau-Start point (1)
float x_start,y_start,theta_start;
//diem lay da - Speed point (2)
float xt,yt;
//vi tri bong - Ball point (3)
float xB,yB;
//toa do giao diem
float xR,yR;
// cap nhat vi tri
float xM,yM,thetaM;
float pos=0;  //sai so e2
//he so lyapunov
float k1 = 1   ;
float k2 = 1 ;
float k3 = 1 ;
// Dai luong mong muon
float vR = 400  ;       // Toc do tham chieu can dat.
float wR = 0.1  ;       // Toc do goc                             
float dt = 0.01 ;       // Thoi gian lay mau 
// Thong so co ban cua robot
uint8_t r = 58/2  ;     // Ban kinh banh xe.
uint8_t R = 90    ;     // ban kinh khung xe
uint8_t dw = 13    ;     // 1 nua do day banh xe
//sai so
float rho,beta;         // sai so khoang cach theo chieu xB (e1)
float alpha;            // sai so goc (e3)
float e2 = 0;           // sai so khoang cach theo chieu yB
float e3 = 0;           // sai so goc cua mat da
float e1 =0;
//controller
float pre_e2=0;
float inte2=0;
//output 
float w1=0,w2=0,w3=0;   //van toc goc 3 banh xe
//xet case
uint8_t step=0;
uint8_t t1=0;
uint8_t path_num=0;

// void receive_value(void)
// {
    
// }

void path(float x,float y,float theta,float x1, float y1)
{
    xB=x1;
    yB=y1;
    theta_start=thetabot;
    x_start=xbot;
    y_start=ybot;
    xt = 1200-(1200-xB)*1.4; //1200-(1200-xB)*1.2
    yt = yB*1.4 ; //yB*1.2
    a1 = (yt-y_start)/(xt-x_start);
    b1 = y_start-a1*x_start;
    a2 = (yB-yt)/(xB-xt);
    b2 = yB-a2*xB;
    //y1 = a1*x1+b1;
    //y2 = a2*x2+b2;
    theta12 = atan2(yt-y_start,xt-x_start); // goc lech doan 12 so voi truc x
    rho12    = sqrt((xt-x_start)*(xt-x_start)+(yt-y_start)*(yt-y_start)) ; // Khoang cach tu 1 -> 2  
    alpha12 = -theta_start + theta12; // goc lech vector v so voi 12
    theta23 = atan2(yB-yt,xB-xt) ; //goc lech doan 23 so voi truc x
    rho23   = sqrt((xB-xt)*(xB-xt)+(yB-yt)*(yB-yt));
    alpha23 = -theta_start + theta23; // goc lech vector v so voi 23
    if      (alpha23>pi/3)      theta0 = 2*pi/3;
    else if (alpha23<-pi/3)     theta0 = -2*pi/3;
    else                        theta0 = 0;
    rho=rho12;
}

void giaodiem(float xM,float yM, float thetaM)
{
    /*
    yR=a1*xR+b1;
    (xR-xM),(yR-yM)
    (1,a1)
    (xR-xM)+a1*(yR-yM)=0

    */
    // a3=tan(alpha12-pi/2);
    // b3=yM-a3*xM;
    // xR=(b3-b1)/(a1-a3);
    xR=(xM-b1+a1*yM)/(1+a1*a1);
    yR=a1*xR+b1;
    pos=sqrt((xR-xM)*(xR-xM)+(yR-yM)*(yR-yM));
    if (abs(pos)<0.01) pos=0;
    if (a1*xM+b1-yM <0) pos = -pos;
    // printf("e2: %f\n", pos);
}
void calculate_error(void)
{
     rho = sqrt((xt-xbot)*(xt-xbot)+(yt-ybot)*(yt-ybot));
    // rho = sqrt((xt-xbot)*(xt-xbot)+(yt-ybot)*(yt-ybot));
    e1 =  sqrt((xt-xR)*(xt-xR)+(yt-yR)*(yt-yR));                                                                        
    e2 = pos;
    // e3 =  atan2(yt-y_start,xt-x_start) - thetabot;
    //e1 =  (xR - xbot)* cos(thetabot) + (yR - ybot)*sin(thetabot);                                                                        
    // e2 = -(xR - xbot)* sin(thetabot) + (yR - ybot)*cos(thetabot);
    e3 =  theta12 -thetabot-alpha12;
    v = vR*cos(e3)+k1*e1;
    w = k2*vR*e2/1000 + k3*sin(e3);
    // if (theta_start <= (atan2(yt-y_start,xt-x_start)+pi/2) && theta_start > (atan2(yt-y_start,xt-x_start)-pi/2))
    // {	
    // w = k1*e2 + k3*(e2-pre_e2)/dt + k2*inte2;
    // }
    // else {w = -(k1*e2 + k3*(e2-pre_e2)/dt + k2*inte2); } 
    vf=v*cos(alpha12);
    vn=v*sin(alpha12);
    printf("w: %f rho: %f\n", w,rho);

}

void position(void)
{
    //printf("xstart: %.2f, ystart: %.2f, thetastart: %.2f\n", x_start, y_start, theta_start);
    if (rho>10 && step==0) step=0;
    else
    {
        if ((abs(e3)>=(pi/36)) && step !=2) step=1;
        else step=2;
    }
    if (t1>100) step=3;   
    switch (step)
    {
        case 0:
        {
            giaodiem(xbot,ybot,thetabot);
            calculate_error();
            break;
        }
        case 1:
        {
            e3 = -thetabot-theta0+theta23;
            vf=0;
            vn=0;
            if (e3>0) w = pi/18;
            else w = -pi/18;
            break;
        }
        case 2:
        {
            t1++;                  
            if (t1<1)
            {
                vf=-vf;
                vn=-vn;
                w=-w;
            }
            if (t1<3)
            {
                vf=0;
                vn=0;
                w=0;
            }
            else if (t1<15) 
            {
                vf=300*cos(theta0);
                vn=300*sin(theta0);
            }
            else if (t1<time_kicking-2) 
            {
                vf=800*cos(theta0);
                vn=800*sin(theta0);
            }            
            else
            {
                vf=300*cos(theta0);
                vn=300*sin(theta0);
            } 
            break;
        }
        case 3:
        {       
            vf=0;
            vn=0;
            w=0;
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
    if (v1 < 0) dir1 = '1'; else dir1 = '0';
    if (v2 < 0) dir2 = '1'; else dir2 = '0'; 
    if (v3 < 0) dir3 = '1'; else dir3 = '0';

    v1 = abs(v1)*30/(r*pi);
    v2 = abs(v2)*30/(r*pi);
    v3 = abs(v3)*30/(r*pi);
    
    printf("w1: %.2f, w2: %.2f, w3: %.2f\n", v1, v2, v3);

    sprintf(vel_pub, "%c%d%c%d%c%d", dir1, int(round(v1+100)), dir2, int(round(v2+100)), dir3, int(round(v3+100)));
    if (count==10)
    {
        printf("vel_pub: %s\n", vel_pub);
        count=0;
    } 
}

void modesubcallback(const std_msgs::UInt8::ConstPtr& msg)
{
    if (msg->data == '9')
    {
        step=0;
        path_num=0;
        e2=0;
        pre_e2=0;
        inte2=0;
        e3=0;
        t1=0;
    }
}

void datasubcallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //read position
    //printf("data: %f\n", msg->data);
    // receive_value();
    xbot        = 1200-float(msg->data[0]);
    //xbot        = 1200-xbot;
    ybot        = float(msg->data[1])-600;
    //ybot        = ybot-600;
    thetabot    = float(msg->data[2])*pi/180;
    xball       = 1200-float(msg->data[3]);
    yball       = float(msg->data[4])-600;
    if (path_num==0)
    {
        path(xbot,ybot,thetabot,xball,yball);
        path_num=1;
    }
    
    //time_delay = 0.5*100/vel_linear;
    printf("bot: %f %f %f %f %f \n", xbot,ybot,thetabot,xball,yball);
    printf("kicking point: %f %f %d %f %f\n", xt,yt,step,theta0*180/pi,alpha12*180/pi);
    position();
    vel_calculate(&v1, &v2, &v3);
    vel_convert(vel_pub);
    //printf( "Receive success\n");
    serialPuts(fd, vel_pub);
    fflush(stdout);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "test_wiringpi_spi");
    ros::NodeHandle node;
    //receive_value();
    //printf("data: %f\n", msg->data);
    //path(xbot,ybot,thetabot,xball,yball);
    // vn = 900*sin(120*pi/180);
    // vf = 900*cos(120*pi/180);
    // w = 0;
    // vel_calculate(&v1, &v2, &v3);
    // vel_convert(vel_pub);
    
    // printf( "Receive success\n");
    // serialPuts(fd, vel_pub);
    
    // fflush(stdout);
    // ros::Duration(0.5).sleep(); 
    // vn = 400*sin(120*pi/180);
    // vf = 400*cos(120*pi/180);
    // w = 0;
    // vel_calculate(&v1, &v2, &v3);
    // vel_convert(vel_pub);
    
    // printf( "Receive success\n");
    // serialPuts(fd, vel_pub);
    
    // fflush(stdout);
    // ros::Duration(0.5).sleep(); 
    // serialPuts(fd, "000000000000");
    //     fflush(stdout);
    if (fd == -1){
        ROS_INFO("Failed");
        return -1;
    }

        
    ROS_INFO("Success");
    
    // fflush(stdout);
    ros::Subscriber data_sub = node.subscribe("data_topic", 1000, datasubcallback);
    ros::Subscriber mode_sub = node.subscribe("mode_topic", 1000, modesubcallback);
    ros::spin();
}

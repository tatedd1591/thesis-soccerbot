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

#define SIZE_OF_ARRAY(array) (sizeof(array)/sizeof(array[0]))
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
char vel_pub[13];
char vel_sub[13];

// MAPPING
float theta0;                   // Chon mat sut bong
float theta12,theta23;          // Goc lech doan 12 va 23 so voi truc X
float alpha12,alpha23;          // Goc lech doan 12,23 so voi xB
float rho12,rho23,rho34;        // Khoang cach doan 12,23,34
float dis=200;                  // Khoang cach lay da
float rate;                     // Ti le khoang cach lay da

//GIA TRI TRUYEN (ROS TO STM)
double vf=0,vn=0,w=0,v=0;       // Van toc tiep, phap, goc va van toc dai trong toa do robot

//VI TRI
// Diem bat dau-Start point (1)
float x_start,y_start,theta_start;
// Diem lay da - Speed point (2)
float xt,yt;
// Vi tri bong - Ball point (3)
float xB,yB;
// Diem trung gian
float xp[5],yp[5];
float xA,yA,xB1,yB1,xC,yC,xD,yD;  // Vi tri trung gian
float dp=150;
uint8_t countp;

// Cap nhat vi tri
float xbot,ybot,thetabot;       // Vi tri robot
float xball,yball;              // Vi tri bong

// SAI SO
float xerror,yerror;            // Vi tri can tinh sai so
// Sai so
float e1 = 0;                   // Sai so khoang cach
float e2 = 0;                   // Sai so goc vector van toc voi quy dao
float e3 = 0;                   // Sai so goc mat sut robot mong muon
// He so e3
float kpE3 = 3   ;
float kdE3 = 0 ;
float kiE3 = 0 ;
float pre_e3=0;
float ipartE3=0;
// He so e1
float kpE1 = 2.5   ;
float kdE1 = 0 ;
float kiE1 = 0 ;
float pre_e1=0;
float ipartE1=0;

// DAI LUONG MONG MUON
float vR   ;                    // Toc do tham chieu can dat.
float wR;                       // Toc do goc                             
float dt = 0.02 ;               // Thoi gian lay mau 

// THONG SO CO BAN CUA ROBOT
uint8_t r = 58/2  ;             // Ban kinh banh xe.
uint8_t R = 90    ;             // Ban kinh khung xe
uint8_t dw = 13   ;             // 1 nua do day banh xe

// OUTPUT 
float w1 = 0, w2 = 0, w3 = 0;   // Van toc goc 3 banh xe

// XET CASE
uint8_t step = 0;
uint8_t t1 = 0;
uint8_t path_num = 0;           // Chon mapping
uint8_t time_kicking = 28;      // 700->15 /800 -> 13 
uint8_t count=0;

void path(float x,float y,float theta,float x1, float y1)
{
    // Vi tri bat dau
    xB=x1;
    yB=y1;
    theta_start=thetabot;
    x_start=xbot;
    y_start=ybot;
    // Diem trung gian
    xA = xB - dp;   yA = yB - dp;
    xB1 = xB - dp;   yB1 = yB + dp;
    xC = xB + dp;   yC = yB + dp;
    xD = xB + dp;   yD = yB - dp;
    // Vi tri lay da
    rho34=sqrt((1200-xB)*(1200-xB)+(yB)*(yB));              // Khoang cach tu bong den tam khung thanh
    rate=(rho34+dis)/rho34;   
    xt = 1200-(1200-xB)*rate;                               // Toa do x
    yt = yB*rate;                                           // Toa do y
    // Start -> diem lay da (12)
    theta12 = atan2(yt-y_start,xt-x_start);                 // Goc lech doan 12 so voi truc x
    rho12   = sqrt((xt-x_start)*(xt-x_start)+(yt-y_start)*(yt-y_start)) ; // Khoang cach tu 1 -> 2  
    alpha12 = -theta_start + theta12;                       // Goc lech vector v so voi 12
    // Diem lay da -> bong (23)
    theta23 = atan2(0-yB,1200-xB) ;                         // Goc lech doan 23 so voi truc x
    rho23   = sqrt((xt-xB)*(xt-xB)+(yt-yB)*(yt-yB));
    alpha23 = -theta_start + theta23;                       // Goc lech vector v so voi 23
    // Chon mat sut bong
    if      (alpha23>pi/3)      theta0 = 2*pi/3;            // Mat 2 ( Doi dien dong co 2 )
    else if (alpha23<-pi/3)     theta0 = -2*pi/3;           // Mat 3 ( Doi dien dong co 3 )
    else                        theta0 = 0;                 // Mat 1 ( doi dien dong co 1 )
    
    xerror = xt;
    yerror = yt;
    // e1 = sqrt((xt-x_start)*(xt-x_start)+(yt-y_start)*(yt-y_start));
    // e2 = -thetabot+atan2(yt-y_start,xt-x_start);                                                                        
    // e3 = -thetabot-theta0+atan2(yB-yt,xB-xt);
    e1 = sqrt((xerror-x_start)*(xerror-x_start)+(yerror-y_start)*(yerror-y_start));
    e2 = -thetabot+atan2(yerror-y_start,xerror-x_start);                                                                        
    //e3 = -thetabot-theta0+atan2(yball-yt,xball-xt);
    e3 = -thetabot-theta0+atan2(0-y_start,1200-x_start);
}

void calculate_error(void)
{
    pre_e3=e3;
    e1 = sqrt((xerror-xbot)*(xerror-xbot)+(yerror-ybot)*(yerror-ybot));
    e2 = -thetabot+atan2(yerror-ybot,xerror-xbot);                                                                        
    //e3 = -thetabot-theta0+atan2(yball-yt,xball-xt);
    e3 = -thetabot-theta0+atan2(0-ybot,1200-xbot);
    printf("e1: %.2f, e2 %.2f, e3: %.2f \n", e1, e2*180/pi, e3*180/pi);
}
void velbot(void)
{   
    //ipartE1=ipartE1+(e1+pre_e1)*dt;
    vR = kpE1*e1+kiE1*ipartE1+kdE1*(e1-pre_e1)/dt;
    if (e1>180)         vR=500;
    // else                vR=kpE1*e1;
    
    //ipartE3=ipartE3+(e3+pre_e3)*dt;
    w = kpE3*e3+kiE3*ipartE3+kdE3*(e3-pre_e3)/dt;
    if (w>pi/6)       w = pi/6;
    else if (w<-pi/6) w = -pi/6;
 
    vf=vR*cos(e2);
    vn=vR*sin(e2);
    printf("w: %f e1: %f\n", w,e1);
    std::stringstream ss;
    ss << "Bot:"<< float(e1) << ":" << float(e2*180/pi) << ":" << float(e3*180/pi);
    error_msg.data = ss.str();
    
    // printf("vel_pub: %s\n", vel_pub);
}

void position(void)
{
    //printf("xstart: %.2f, ystart: %.2f, thetastart: %.2f\n", x_start, y_start, theta_start);
    // if ((e1>5 || e2 > pi/180)&& step==0) step=0;
    // else step=1; 
//0 -> -37
//37 ->0
//1200-35 ->1200
// 200 -> 177
// 
    if (step==0) calculate_error();
    if (t1>time_kicking) step=2;
  
    switch (step)
    {
        case 0:
        {
            // calculate_error();
            if ((abs(e1)>10 || abs(e3) > pi/180)) { velbot();   break;  }
            else 
            {
                // if ((w>0 && e3<-0.5*pi/180) || (w<0 && e3>0.5*pi/180) )
                // {
                //     // vf=-vf;
                //     // vn=-vn;
                //     w=-w;
                    
                // }
                // else 
                // {
                    vf=0;
                    vn=0;
                    w=0;
                // }
                step=1; 
            } 
            //break;
        }
        case 1:
        {
            //B1: dung 0.08s
            vf=0;
            vn=0;
            w=0;
            vel_calculate(&v1,&v2, &v3);
            vel_convert(vel_pub);
            serialPuts(fd, vel_pub);
            fflush(stdout);
            ros::Duration(0.08).sleep();           
            
            //B2: chay da 0.35s
            vf=300*cos(theta0);
            vn=300*sin(theta0);
            vel_calculate(&v1,&v2, &v3);
            vel_convert(vel_pub);
            serialPuts(fd, vel_pub);
            fflush(stdout);
            ros::Duration(0.35).sleep();
            
            //B3: Tang toc 0.3s
            vf=800*cos(theta0);
            vn=800*sin(theta0);
            vel_calculate(&v1,&v2, &v3);
            vel_convert(vel_pub);
            serialPuts(fd, vel_pub);
            fflush(stdout);
            ros::Duration(0.3).sleep();
            
            //B4: Giam toc 0.1s
            vf=300*cos(theta0);
            vn=300*sin(theta0);
            vel_calculate(&v1,&v2, &v3);
            vel_convert(vel_pub);
            serialPuts(fd, vel_pub);
            fflush(stdout);
            ros::Duration(0.1).sleep(); 
            step=2;                                       
            break;
        }
        case 2:
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
    count++;
    printf("w1: %.2f, w2: %.2f, w3: %.2f\n", v1, v2, v3);
    if (v1 < 0) dir1 = '1'; else dir1 = '0';
    if (v2 < 0) dir2 = '1'; else dir2 = '0'; 
    if (v3 < 0) dir3 = '1'; else dir3 = '0';

    v1 = (abs(v1)*30)/(r*pi);
    v2 = (abs(v2)*30)/(r*pi);
    v3 = (abs(v3)*30)/(r*pi);
      
    sprintf(vel_pub, "%c%d%c%d%c%d", dir1, int(round(v1+100)), dir2, int(round(v2+100)), dir3, int(round(v3+100)));
    if (count==5)
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
        e1=0;
        e3=0;
        t1=0;
    }
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
    
    //check dong co
    // w=pi;

    // Tinh toan van toc dong co 
    vel_calculate(&v1, &v2, &v3);
    vel_convert(vel_pub);
    // Gui du lieu gui ve STM
    serialPuts(fd, vel_pub);
    fflush(stdout);
    // receive
    do
    {
		vel_sub = serialGetchar(fd);
		printf("%c",vel_sub);
		fflush (stdout);
	}
    while(serialDataAvail(fd));
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
    ros::Publisher error_pub = node.advertise<std_msgs::String>("/error_pub/primitive", 1000);
    ros::Subscriber data_sub = node.subscribe("data_topic", 1000, datasubcallback);
    ros::Subscriber mode_sub = node.subscribe("mode_topic", 1000, modesubcallback);
    while (ros::ok())
    {
        error_pub.publish(error_msg);
        ros::Duration(0.02).sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

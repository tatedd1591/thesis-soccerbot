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

// Vi tri cac doi tuong
struct Object
{
    float x;
    float y;
    float theta;
};
//void convert(char msg->data);
const double pi = 3.1415926;
int fd = serialOpen("/dev/ttyAMA0", 115200);
double v1 = 0, v2 = 0, v3 = 0;
uint8_t dir0=2;
char vel_pub[14];
char vel_sub[13];
int a=0;
// MAPPING
float theta0;                   // Chon mat sut bong
float theta12,theta23;          // Goc lech doan 12 va 23 so voi truc X
float alpha12,alpha23;          // Goc lech doan 12,23 so voi vector v
float rho12,rho23,rho34;        // Khoang cach doan 12,23,34
float dis=200;                  // Khoang cach lay da
float rate;                     // Ti le khoang cach lay da

// Diem trung gian
float xp[3],yp[3];              // Diem trung gian
float dp=200;                   // Mien han che
float countp;                   // So diem trung gian

//GIA TRI TRUYEN (ROS TO STM)
double vf=0,vn=0,w=0,v=0;       // Van toc tiep, phap, goc va van toc dai trong toa do robot

// Toa do sai so
float xerror,yerror;            // Vi tri can tinh sai so
// SAI SO VA PID
struct PID
{
    float kp;
    float ki;
    float kd;
    float e;
    float pre_e;
    float ipart;
};
PID PID_e1 = {2.5,0,0,0,0,0};   // Sai so khoang cach
PID PID_e2 = {0,0,0,0,0,0};     // Sai so goc vector van toc voi quy dao
PID PID_e3 = {3,0,0,0,0,0};     // Sai so goc mat sut robot mong muon

// DAI LUONG MONG MUON
float vR=500    ;               // Toc do tham chieu can dat.
float wR        ;               // Toc do goc                             
float dt = 0.04 ;               // Thoi gian lay mau 

// THONG SO CO BAN CUA ROBOT
uint8_t r = 58/2  ;             // Ban kinh banh xe.
uint8_t R = 90    ;             // Ban kinh khung xe
uint8_t dw = 13   ;             // 1 nua do day banh xe

// OUTPUT 
float w1 = 0, w2 = 0, w3 = 0;   // Van toc goc 3 banh xe

// XET CASE
uint8_t step = 0;               // TH di chuyen
bool path_num = 0;              // Chon mapping

// GIA TRI TUYEN (ROS TO MQTT)
std_msgs::String error_msg;
ros::Publisher error_pub;

void modesubcallback(const std_msgs::UInt8::ConstPtr& msg);
void datasubcallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void receive_value(Object* bot, Object* ball);
void vel_calculate(double *v1, double *v2, double *v3);
void vel_convert(char* vel_pub);
void path(Object* bot, Object* ball, Object* target);
void calculate_error(Object* bot, Object* ball, Object* target);
void velbot(void);
void position(Object* bot, Object* ball, Object* target);

void receive_value(Object* bot, Object* ball)
{
    bot->x        = 1200-float(msg->data[0]);
    bot->y        = float(msg->data[1])-600;
    bot->theta    = float(msg->data[2])*pi/180;
    ball->x       = 1200-float(msg->data[3]);
    ball->y       = float(msg->data[4])-600;
}

void path(Object* bot, Object* ball, Object* target)
{ 
    // Diem trung gian (A,B,C,D)
    Object A_point, B_point, C_point, D_point;
    A_point.x = ball->x - dp;   A_point.y = ball->y - dp;
    B_point.x = ball->x - dp;   B_point.y = ball->y + dp;
    C_point.x = ball->x + dp;   C_point.y = ball->y + dp;
    D_point.x = ball->x + dp;   D_point.y = ball->y - dp;
    // Vi tri lay da
    rho34=sqrt((1200-ball->x)*(1200-ball->x)+(ball->y)*(ball->y));// Khoang cach tu bong den tam khung thanh
    rate=(rho34+dis)/rho34;   
    target->x = 1200-(1200-ball->x)*rate;                         // Toa do x
    target->y = ball->y*rate;                                     // Toa do y
    // Start -> diem lay da (12)
    theta12 = atan2(target->y-bot->y,target->x-bot->x);           // Goc lech doan 12 so voi truc x
    rho12   = sqrt((target->x-bot->x)*(target->x-bot->x)+(target->y-bot->y)*(target->y-bot->y)) ; // Khoang cach tu 1 -> 2  
    alpha12 = -bot->theta + theta12;                              // Goc lech vector v so voi 12
    // Diem lay da -> bong (23)
    theta23 = atan2(0-ball->y,1200-ball->x) ;                     // Goc lech doan 23 so voi truc x
    rho23   = sqrt((target->x-ball->x)*(target->x-ball->x)+(target->y-ball->y)*(target->y-ball->y));
    alpha23 = -bot->theta + theta23;                              // Goc lech vector v so voi 23
    // Chon mat sut bong
    if      (alpha23>pi/3)      theta0 = 2*pi/3;                  // Mat 2 ( Doi dien dong co 2 )
    else if (alpha23<-pi/3)     theta0 = -2*pi/3;                 // Mat 3 ( Doi dien dong co 3 )
    else                        theta0 = 0;                       // Mat 1 ( doi dien dong co 1 )
    // Xet cac diem can den cho toi diem lay da
    if (bot->x <= target->x)                                      // Robot nam truoc diem lay da theo chieu x
    {
        countp=0;
        xerror = target->x;    yerror = target->y;
    }
    else                                                          // Robot nam sau diem lay da theo chieu x
    {
        if (abs(bot->x - ball->x) < dp && abs(bot->y - ball->y) < dp)   // Robot nam trong mien han che
        {
            if (bot->x > ball->x)                                 // Robot nam sau bong theo chieu x
            {
                countp=2;
                // Robot gan diem D hon
                if (sqrt((bot->x - C_point.x)*(bot->x - C_point.x)+(bot->y - C_point.y)*(bot->y-C_point.y)) > sqrt((bot->x-D_point.x)*(bot->x-D_point.x)+(bot->y-D_point.y)*(bot->y-D_point.y)))  
                {
                    xp[0]=D_point.x;   yp[0]=D_point.y;
                    xp[1]=A_point.x;   yp[1]=A_point.y;
                }
                else                                              // Robot nam gan diem C hon
                {
                    xp[0]=C_point.x;   yp[0]=C_point.y;
                    xp[1]=B_point.x;   yp[1]=B_point.y;
                }
            }
            else                                                  // Robot nam truoc bong theo chieu x
            {
                countp=1;
                //Robot gan diem A hon
                if (sqrt((bot->x-B_point.x)*(bot->x-B_point.x)+(bot->y-B_point.y)*(bot->y-B_point.y)) > sqrt((bot->x-A_point.x)*(bot->x-A_point.x)+(bot->y-A_point.y)*(bot->y-A_point.y)))
                {
                    xp[0]=A_point.x;   yp[0]=A_point.y;
                }
                else                                              // Robot nam gan diem B hon
                {
                    xp[0]=B_point.x;   yp[0]=B_point.y;
                }
            }
        }                                       
        else                                                      // Robot nam ngoai mien han che
        {
            if (abs(bot->y-B_point.y)>=dp)                        // Robot nam ngoai mien gia tri y cua mien han che
            {
                countp=1;
                if (bot->y<= B_point.y - dp)                      // Robot nam duoi mien han che
                {
                    xp[0]=A_point.x;   yp[0]=A_point.y;
                }
                else if (bot->y>= B_point.y + dp)                 // Robot nam tren mien han che
                {
                    xp[0]=B_point.x;  yp[0]=B_point.y;
                }
            }
            else                                                  // Robot nam trong mien gia tri y cua mien han che
            {
                countp=2;
                if (bot->y >= B_point.y)                          // Robot nam tren bong
                {    
                    if (C_point.y > 350)                          // Diem trung gian phia tren ngoai vung di chuyen robot
                    {
                        xp[0]=D_point.x;   yp[0]=D_point.y;
                        xp[1]=A_point.x;   yp[1]=A_point.y;
                    }
                    else                                          // Diem trung gian trong vung di chuyen robot
                    {
                        xp[0]=C_point.x;   yp[0]=C_point.y;
                        xp[1]=B_point.x;   yp[1]=B_point.y;
                    }
                }
                else                                              // Robot nam duoi bong
                {
                    if (D_point.y < -350)                         // Diem trung gian phia duoi ngoai vung di chuyen robot
                    {                   
                        xp[0]=C_point.x;   yp[0]=C_point.y;
                        xp[1]=B_point.x;   yp[1]=B_point.y;
                    }
                    else                                          // Diem trung gian trong vung di chuyen robot
                    {                   
                        xp[0]=D_point.x;   yp[0]=D_point.y;
                        xp[1]=A_point.x;   yp[1]=A_point.y;
                    }
                }
            }
        }
        xerror = xp[0];     yerror = yp[0];
    }
    PID_e1.e = sqrt((xerror-bot->x)*(xerror-bot->x)+(yerror-bot->y)*(yerror-bot->y));
    PID_e2.e = -bot->theta+atan2(yerror-bot->y,xerror-bot->x);                                                                        
    PID_e3.e = -bot->theta-theta0+atan2(0-bot->y,1200-bot->x);
}

void calculate_error(Object* bot, Object* ball, Object* target)
{
    PID_e3.pre_e=PID_e3.e;
    PID_e1.e = sqrt((xerror-bot->x)*(xerror-bot->x)+(yerror-bot->y)*(yerror-bot->y));
    PID_e2.e = -bot->theta+atan2(yerror-bot->y,xerror-bot->x);                                                                        
    PID_e3.e = -bot->theta-theta0+atan2(0-bot->y,1200-bot->x);
    printf("e1: %.2f, e2 %.2f, e3: %.2f \n", PID_e1.e, PID_e2.e*180/pi, PID_e3.e*180/pi);
}
void velbot(void)
{   
    // PID_e1.ipart=PID_e1.ipart+(PID_e1.e+PID_e1.pre_e)*dt;
    vR = PID_e1.kp*PID_e1.e+PID_e.ki*PID_e1.ipart+PID_e3.kd*(PID_e1.e-PID_e1.pre_e)/dt;
    if (PID_e1.e>200)         vR=500;
    
    // PID_e3.ipart=PID_e3.ipart+(PID_e3.e+PID_e3.pre_e)*dt;
    w = PID_e.kp*PID_e3.e+PID_e3.ki*PID_e3.ipart+PID_e3.kd*(PID_e3.e-PID_e3.pre_e)/dt;
    if (w>pi/6)       w = pi/6;
    else if (w<-pi/6) w = -pi/6;
 
    vf=vR*cos(PID_e2.e);
    vn=vR*sin(PID_e2.e);
    printf("w: %f e1: %f\n", w,PID_e1.e);
    std::stringstream ss;
    ss << "Bot:"<< float(PID_e1.e) << ":" << float(PID_e2.e*180/pi) << ":" << float(PID_e3.e*180/pi);
    error_msg.data = ss.str();
}

void position(Object* bot, Object* ball, Object* target)
{
    if (step==0) calculate_error(&bot,&ball,&target);       // Chi tinh sai so trong step 0
    switch (step)
    {
        case 0:                                             // Di chuyen den diem lay da
        {
            // calculate_error();
            if (countp==0)                                  // ko con diem trung gian
            {
                if ((abs(PID_e1.e)>10 || abs(PID_e3.e) > pi/360))       // Chua dat den sai so chap nhan
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
            else                                            // Con diem trung gian
            {
                if (abs(PID_e1.e)<50)                             // Den diem trung gian
                {
                    countp--;
                    if (countp==0)                          // Ko con diem trung gian -> den diem lay da   
                    {
                        xerror=target->x;  yerror=target->y;
                    }
                    else                                    // Con diem trung gian -> den diem trung gian thu 2
                    {
                        xerror=xp[1];   yerror=yp[1];       
                    }
                }
                else    velbot();                           // Chua den diem trung gian
                break;
            }           
        }
        case 1:                                             // Thuc hien sut bong
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
        case 2:                                                 // Ket thuc chuong trinh
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
    printf("w1: %.2f, w2: %.2f, w3: %.2f\n", v1, v2, v3);
    if (v1 < 0) dir1 = '1'; else dir1 = '0';
    if (v2 < 0) dir2 = '1'; else dir2 = '0'; 
    if (v3 < 0) dir3 = '1'; else dir3 = '0';

    v1 = (abs(v1)*30)/(r*pi);
    v2 = (abs(v2)*30)/(r*pi);
    v3 = (abs(v3)*30)/(r*pi);
      
    sprintf(vel_pub, "%c%d%c%d%c%d%d", dir1, int(round(v1+100)), dir2, int(round(v2+100)), dir3, int(round(v3+100)),dir0);
    printf("vel_pub: %s\n", vel_pub);
}

void modesubcallback(const std_msgs::UInt8::ConstPtr& msg)
{
}

void datasubcallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    Object bot_obj,ball_obj,target_obj;
    // Doc vi tri robot
    receive_value(&bot_obj,&ball_obj);
    // Tao quy dao
    if (path_num==0)
    {
        path(&bot_obj,&ball_obj,&target_obj);
        path_num=1;
    }
    // Kiem tra vi tri robot va diem sut bong
    printf("bot: %f %f %f %f %f \n", bot_obj.x,bot_obj.y,bot_obj.theta*180/pi,ball_obj.x,ball_obj.y);
    printf("kicking point: %f %f %d %f %f\n", target_obj.x,target_obj.y,step,theta0*180/pi,alpha12*180/pi);
    // Xet TH robot di chuyen
    position(&bot_obj,&ball_obj,&target_obj);
    // Tinh toan van toc dong co 
    vel_calculate(&v1, &v2, &v3);
    vel_convert(vel_pub);
    // Gui du lieu gui ve STM
    serialPuts(fd, vel_pub);
    fflush(stdout);
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
        ros::Duration(0.04).sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

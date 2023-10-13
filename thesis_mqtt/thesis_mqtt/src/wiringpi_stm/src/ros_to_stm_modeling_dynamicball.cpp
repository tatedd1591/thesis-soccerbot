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
ros::Publisher error_pub;
#define SIZE_OF_ARRAY(array) (sizeof(array)/sizeof(array[0]))

//void convert(char msg->data);
const double pi = 3.1415926;
int fd = serialOpen("/dev/ttyAMA0", 115200);
double v1 = 0, v2 = 0, v3 = 0;
uint8_t dir0=2;
char vel_pub[14];
char vel_sub;

// MAPPING                              
float theta0;                                   // Chon mat sut bong
float alpha23;                                  // Goc lech giua huong bong so voi robot
float dis=150;                                  // Khoang cach robot do bong
float rate;                                     // Khoang cach giua robot va bong

//GIA TRI TRUYEN (ROS TO STM)
double vf=0,vn=0,w=0,v=0;                       // Van toc tiep, phap, goc va van toc dai trong toa do robot

// Vi tri cac doi tuong
struct Object
{
    float x;
    float y;
    float theta;
    float velocity;
    float pre_v;
    float pre_x[4],pre_y[4];                // Vi tri bong tai 4 diem gan nhat
};

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
PID PID_e1 = {2.5,0,0,0,0,0};                     // Sai so khoang cach
PID PID_e2 = {0,0,0,0,0,0};                       // Sai so goc vector van toc voi quy dao
PID PID_e3 = {3,0,0,0,0,0};                       // Sai so goc mat sut robot mong muon

// BALL MOVING
float sumx=0,sumy=0,sumx2=0,sumy2=0,sumxy=0;    // Tong cac diem x va y
float sxx=0,sxy=0,syy=0;                        // Tong binh phuong sua doi
float a1,b1;                                    // Duong di chuyen cua bong: y=a1*x+b1;
float a11,b11;                                  // He so calib
// Du doan
float deltat;                                   // Thoi gian robot toi truoc bong
float at,bt,ct;                                 // Phuong trinh du doan thoi gian
float tpredict;                                 // Thoi gian du doan
float num;                                      // Bien trung gian
uint8_t counttheta=0;                           
float distance=0;

// THONG SO CO BAN CUA ROBOT
uint8_t r = 58/2  ;                             // Ban kinh banh xe.
uint8_t R = 90    ;                             // Ban kinh khung xe
uint8_t dw = 13   ;                             // 1 nua do day banh xe

// DAI LUONG MONG MUON
float vR   = 500  ;                             // Toc do tham chieu can dat.
float wR   = pi/6 ;                             // Toc do goc
float vC          ;                             //= 200;
float RC   = 65  ;                              // Ban kinh xoay
float wC   = pi/6 ;                             //= vC/RC: Toc do goc                                                   
float dt   = 0.04 ;                             // Thoi gian lay mau 

// OUTPUT 
float w1 = 0, w2 = 0, w3 = 0;                   // Van toc goc 3 banh xe

// XET CASE
uint8_t step = 0  ;

void modesubcallback(const std_msgs::UInt8::ConstPtr& msg);
void datasubcallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void receive_value(Object* bot, Object* ball);
void vel_calculate(double *v1, double *v2, double *v3);
void vel_convert(char* vel_pub);
void path(Object* bot, Object* ball);
void calculate_error(Object* bot, Object* ball, Object* target);
void velbot(void);
void position(Object* bot, Object* ball, Object* target);
void ball_moving(Object* bot, Object* ball);
void predict_pos(Object* bot, Object* ball, Object* target);
void predict_time(Object* bot, Object* ball);

// GIA TRI TUYEN (ROS TO MQTT)
std_msgs::String error_msg;
std::stringstream ss;
float giaiPTBac2(float a, float b, float c) 
{
    float x;
    // kiem tra cac he so
    if (a == 0) 
    {
        if (b == 0)     x=9999;                 //printf("Phuong trinh vo nghiem!");
        else            x=-c/b;
    }
    else
    {
        // tinh delta
        float delta = b*b - 4*a*c;
        float x1;
        float x2;
        // tinh nghiem
        if (delta > 0) 
        {
            x1 = (float) ((-b + sqrt(delta)) / (2*a));
            x2 = (float) ((-b - sqrt(delta)) / (2*a));
            if (x1<0 && x2<0)   x=0;
            else if (x1<=0)      x=x2;
            else if (x2<=0)      x=x1;
            else
            {
                if (x1>x2)      x=x2;
                else            x=x1;
            }  
        } 
        else if (delta == 0)      x = (-b / (2 * a));
        else                        x=9999;     //printf("Phuong trinh vo nghiem!");            
    }
    return x;
}
void receive_value(Object* bot, Object* ball)
{
    bot->x         = 1200-float(msg->data[0]);
    bot->y         = float(msg->data[1])-600;
    bot->theta     = float(msg->data[2])*pi/180;
    ball->x        = 1200-float(msg->data[3]);
    ball->y        = float(msg->data[4])-600;
    ball->pre_v    = ball->velocity;
    ball->velocity = float(msg->data[5]);
}
void path(Object* bot, Object* ball)
{
    alpha23 = -(bot->theta) + ball->theta; // goc lech vector v so voi 23
    if      (alpha23>pi/3)      theta0 = 2*pi/3;
    else if (alpha23<-pi/3)     theta0 = -2*pi/3;
    else                        theta0 = 0;
}
void ball_moving(Object* bot, Object* ball)
{
    sumx=0;
    sumy=0;
    sumx2=0;
    sumy2=0;
    sumxy=0;
    for (int i=0;i<3;i++)
    {
        ball->pre_x[3-i] = ball->pre_x[2-i];
        ball->pre_y[3-i] = ball->pre_y[2-i];
    }
    ball->pre_x[0] = ball->x;
    ball->pre_y[0] = ball->y;
    for (int j=0;j<4;j++)
    {
        sumx = sumx + ball->pre_x[j];
        sumy = sumy + ball->pre_y[j];
        sumx2 = sumx2 + ball->pre_x[j]*ball->pre_x[j];
        sumy2 = sumy2 + ball->pre_y[j]*ball->pre_y[j];
        sumxy = sumxy + ball->pre_x[j]*ball->pre_y[j];  
    }
    sxx = sumx2 - sumx*sumx/4;
    syy = sumy2 - sumy*sumy/4;
    sxy = sumxy - sumx*sumy/4;
    // y = a1*x+b1 => a1*x-y +b1=0
    a1 = sxy/sxx;
    b1 = sumy/4 - a1*sumx/4;
    a11=(ball->pre_x[3]-ball->pre_x[0])/abs(ball->pre_x[3]-ball->pre_x[0]);
    b11=(ball->pre_y[3]-ball->pre_y[0])/abs(ball->pre_y[3]-ball->pre_y[0]);
    if (counttheta<2) counttheta++;
    else
    {
        ball->theta=atan2(b11*abs(sxy),a11*abs(sxx));
        counttheta=0;
    }
    printf("ball->theta: %.2f VBALL: %.2f\n",ball->theta*180/pi,vball);
    rate=sqrt((ball->x-bot->x)*(ball->x-bot->x)+(ball->y-bot->y)*(ball->y-bot->y));
    distance = (a1*bot->x+b1-bot->y)/sqrt(a1*a1+1);
}
void predict_time(Object* bot, Object* ball)
{
    deltat = 350/ball->velocity;    
    if (ball->theta>0)      num=(ball->x-bot->x)*cos(ball->theta-pi)+(ball->y-bot->y)*sin(ball->theta-pi);
    else                    num=(ball->x-bot->x)*cos(ball->theta+pi)+(ball->y-bot->y)*sin(ball->theta+pi);
    at=500*500-ball->velocity*ball->velocity;
    if (rate<400) at=vR*vR-ball->velocity*ball->velocity;
    bt = -2*(ball->velocity*ball->velocity*deltat+ball->velocity*num);
    ct=-2*ball->velocity*num*deltat-ball->velocity*ball->velocity*deltat*deltat - (ball->x-bot->x)*(ball->x-bot->x) - (ball->y-bot->y)*(ball->y-bot->y);
    tpredict= giaiPTBac2(at,bt,ct);
}
void predict_pos(Object* bot, Object* ball,Object* target)
{
    if (ball->theta>0)
    {
        target->x = ball->x+ball->velocity*tpredict*cos(ball->theta-pi);
        target->y = ball->y+ball->velocity*tpredict*sin(ball->theta-pi);
    }
    else if (ball->theta<0)
    {
        target->x = ball->x+ball->velocity*tpredict*cos(ball->theta+pi);
        target->y = ball->y+ball->velocity*tpredict*sin(ball->theta+pi);
    }
    printf("xt: %.2f yt: %.2f , tpre: %.2f\n",xt,yt,tpredict);
}

void calculate_error(Object* bot, Object* ball, Object* target)
{
    PID_e3.pre_e=PID_e3.e;
    PID_e1.pre_e=PID_e1.e;
    PID_e1.e = sqrt((target->x-bot->x)*(target->x-bot->x)+(target->y-bot->y)*(target->y-bot->y));
    PID_e2.e = -bot->theta+atan2(target->y-bot->y,target->x-bot->x);                                                                        
    if (step==0 || step==1 || step==2) PID_e3.e = -bot->theta-theta0+ball->theta;
    else if (step==3)       PID_e3.e = -bot->theta-theta0+atan2(-75-bot->y,1200-bot->x);
    printf("e1: %.2f, e2 %.2f, e3: %.2f \n", PID_e1.e, PID_e2.e*180/pi, PID_e3.e*180/pi);
    // ss << "Bot:"<< int(round(PID_e1.e)) << ":" << int(round(PID_e2.e*180/pi)) << ":" << int(round(PID_e3.e*180/pi)) << std::endl;
    // error_msg.data = ss.str();
}
void velbot(void)
{   
    // PID_e1.ipart=PID_e1.ipart+(PID_e1.e+PID_e1.pre_e)*dt;
    vR = PID_e1.kp*PID_e1.e+PID_e1.ki*PID_e1.ipart+PID_e1.kd*(PID_e1.e-PID_e1.pre_e)/dt;
    if (vR>500) vR=500;
    // PID_e3.ipart=PID_e3.ipart+(PID_e3.e+PID_e3.pre_e)*dt;
    w = PID_e3.kp*PID_e3.e+PID_e3.ki*PID_e3.ipart+PID_e3.kd*(PID_e3.e-PID_e3.pre_e)/dt;
    if (w>pi/6)       w = pi/6;
    else if (w<-pi/6) w = -pi/6;
 
    vf=vR*cos(PID_e2.e);
    vn=vR*sin(PID_e2.e);
}

void position(Object* bot, Object* ball, Object* target)
{
    //theta0=0;
    if (abs(bot->y) > 500 || (bot->x) < -50) step=5;
    if (step==1) 
    {
        predict_time(&bot,&ball);
        //if (tpredict==9999) step=5;
        predict_pos(&bot,&ball,&target);
    }
    if (step==1 || step==2 || step ==3)    calculate_error(&bot,&ball,&target);  
    //printf("e1: %.2f, e2 %.2f , e3: %2f , dis: %.2f\n",PID_e1.e,PID_e2.e,PID_e3.e,distance);
    switch (step)
    {
        case 0:
        {
            // Phat hien bong dang di chuyen
            if ((ball->velocity) > 120 && (ball->velocity) < 800)    
            {
                //dir0=2;
                predict_time(&bot,&ball);
                if (tpredict==9999) step=5;
                predict_pos(&bot,&ball,&target);
                path(&bot,&ball);
                calculate_error(&bot,&ball,&target);
                dir0=1;
                step=1;
            }
            else    // Bong chua di chuyen
            {
                vf=0;
                vn=0;
                w=0;
            }
            break;
        }
        case 1:
        {
            // Bong trong mien gioi han
            if ((abs(PID_e1.e)<= 100 && abs(PID_e3.e)<pi/60) || rate <=200) 
            {
                vf=0;
                vn=0;
                w=0;
                step=2;
            }
            else    velbot();
            break;         
        }
        case 2:
        {
            // Toi khoang cach do bong ly tuong
            if (rate<dis)
            {
                if (vR<0)
                {
                    // Dung 0.1s
                    vf=0;
                    vn=0;
                    w=0;
                    vel_calculate(&v1,&v2,&v3);
                    vel_convert(vel_pub);
                    serialPuts(fd, vel_pub);
                    fflush(stdout);
                    ros::Duration(0.1).sleep();
                }
                // Lao toi do bong 0.4s
                vf=200*cos(theta0);
                vn=200*sin(theta0);
                w=0;
                vel_calculate(&v1,&v2,&v3);
                vel_convert(vel_pub);
                serialPuts(fd, vel_pub);
                fflush(stdout);
                ros::Duration(0.4).sleep();
                // Dung 0.1s
                vf=0;
                vn=0;
                w=0;
                vel_calculate(&v1,&v2, &v3);
                vel_convert(vel_pub);
                serialPuts(fd, vel_pub);
                fflush(stdout);
                ros::Duration(0.1).sleep(); 
                step=3;
                PID_e3.e = -bot->theta-theta0+atan2(-50-bot->y,1200-bot->x);
            }
            else    velbot();
            break;    
        }
        case 3:
        {
            // Chinh mat da ve khung thanh
            if (abs(PID_e3.e)>pi/180)
            {
                w=PID_e3.kp*PID_e3.e;
                if (w>wC*3)  w=3*wC;
                else if (w<-3*wC)        w=-3*wC;
                vf=w*RC*cos(theta0-pi/2);
                vn=w*RC*sin(theta0-pi/2);          
                break;
            }
            else    step=4;
        }
        case 4:
        {
            //B1: Dung 0.1s
            vf=0;
            vn=0;
            w=0;
            vel_calculate(&v1,&v2, &v3);
            vel_convert(vel_pub);
            serialPuts(fd, vel_pub);
            fflush(stdout);
            ros::Duration(0.1).sleep();
            
            //B2: Lui 0.1s
            vf=-200*cos(theta0);
            vn=-200*sin(theta0);
            w=0;
            vel_calculate(&v1,&v2, &v3);
            vel_convert(vel_pub);
            serialPuts(fd, vel_pub);
            fflush(stdout);
            ros::Duration(0.1).sleep();
            
            //B3: Nang mat da, dung 0.1s
            dir0=2;           
            vf=0;
            vn=0;
            w=0;
            vel_calculate(&v1,&v2, &v3);
            vel_convert(vel_pub);
            serialPuts(fd, vel_pub);
            fflush(stdout);
            ros::Duration(0.2).sleep();
            
            //B4: chay da 0.35s
            vf=300*cos(theta0);
            vn=300*sin(theta0);
            vel_calculate(&v1,&v2, &v3);
            vel_convert(vel_pub);
            serialPuts(fd, vel_pub);
            fflush(stdout);
            ros::Duration(0.3).sleep();
            
            //B5: Tang toc 0.3s
            vf=800*cos(theta0);
            vn=800*sin(theta0);
            vel_calculate(&v1,&v2, &v3);
            vel_convert(vel_pub);
            serialPuts(fd, vel_pub);
            fflush(stdout);
            ros::Duration(0.3).sleep();
            
            //B6: Giam toc 0.1s
            vf=300*cos(theta0);
            vn=300*sin(theta0);
            vel_calculate(&v1,&v2, &v3);
            vel_convert(vel_pub);
            serialPuts(fd, vel_pub);
            fflush(stdout);
            ros::Duration(0.2).sleep(); 
            step=5;                                       
            break;
        }
        case 5:             
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
    
    //printf("w1: %.2f, w2: %.2f, w3: %.2f\n", v1, v2, v3);
    sprintf(vel_pub, "%c%d%c%d%c%d%d", dir1, int(round(v1+100)), dir2, int(round(v2+100)), dir3, int(round(v3+100)),dir0);   
    printf("vel_pub: %s\n", vel_pub);
}

void modesubcallback(const std_msgs::UInt8::ConstPtr& msg)
{
}

void datasubcallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    Object bot_obj, ball_obj, target_obj;
    //read position
    receive_value(&bot_obj, &ball_obj)
    ball_moving(&bot_obj, &ball_obj);
    //printf("ball velocity: %f\n", ball_obj.velocity);
    position(&bot_obj, &ball_obj, &target_obj);
    vel_calculate(&v1, &v2, &v3);
    vel_convert(vel_pub);
    printf("\n");
    //printf( "Receive success\n");
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
    // ros::Publisher error_pub = node.advertise<std_msgs::String>("/error_pub/primitive", 1000);
    ros::Subscriber data_sub = node.subscribe("data_topic", 1000, datasubcallback);
    ros::Subscriber mode_sub = node.subscribe("mode_topic", 1000, modesubcallback);
    while (1)
    {
        // error_pub.publish(error_msg);
        // ros::Duration(0.08).sleep();
        ros::spinOnce();
    }
    ros::spin();
}

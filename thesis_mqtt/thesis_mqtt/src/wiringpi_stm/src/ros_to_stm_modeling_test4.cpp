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
void modesubcallback(const std_msgs::UInt8::ConstPtr& msg);
void datasubcallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void vel_calculate(double *v1, double *v2, double *v3);
void vel_convert(char* vel_pub);
void path(float x,float y,float theta,float x1, float y1);
void calculate_error(void);
void velbot(void);
void position(void);
void receive_value(void);
void ball_moving(void);
void predict_pos(void);
void predict_time(void);

//void convert(char msg->data);
const double pi = 3.1415926;
int fd = serialOpen("/dev/ttyAMA0", 115200);
double v1 = 0, v2 = 0, v3 = 0;
uint8_t dir0=2;
char vel_pub[14];
char vel_sub;
uint8_t countdir=0;
// MAPPING                              
float theta0;                                   // Chon mat sut bong
float alpha23;                                  // Goc lech giua huong bong so voi robot
float dis=200;                                  // Khoang cach lui cua robot
float rate;                                     // Khoang cach giua robot va bong
float vsp;                                      // Van toc lui
//GIA TRI TRUYEN (ROS TO STM)
double vf=0,vn=0,w=0,v=0;                       // Van toc tiep, phap, goc va van toc dai trong toa do robot

//VI TRI
// Diem bat dau-Start point (1)
float x_start,y_start,theta_start;
// Diem lay da - Speed point (2)
float xt,yt;
// Vi tri bong - Ball point (3)
float xB,yB;
// Cap nhat vi tri
float xbot,ybot,thetabot;                       // Vi tri robot
float xball,yball;                              // Vi tri bong
float vrec=0;                                   // Van toc bong nhan duoc
float xp,yp;

// BALL MOVING
float pre_xball[4],pre_yball[4];                // Vi tri bong tai 4 diem gan nhat
float vB[4];                                    // Gia tri van toc bong gan nhat
float sumx=0,sumy=0,sumx2=0,sumy2=0,sumxy=0;    // Tong cac diem x va y
float sxx=0,sxy=0,syy=0;                        // Tong binh phuong sua doi
float a1,b1;                                    // Duong di chuyen cua bong: y=a1*x+b1;
float vball=0;                                  // Van toc bong
float pre_vball=0;                              // Van toc bong truoc do
float thetaball;                                // Huong bong
// Du doan
float deltat;                                   // Thoi gian robot toi truoc bong
float at,bt,ct;                                 // Phuong trinh du doan thoi gian
float tpredict;                                 // Thoi gian du doan
float num;                                      // Bien trung gian
uint8_t countb=0;                               // Thoi gian doc huong bong
uint8_t counttheta=0;                           
float distance=0;

// SAI SO
float xerror,yerror;
// Sai so
float e1 = 0;                                   // Sai so khoang cach
float e2 = 0;                                   // Sai so goc vector van toc voi quy dao
float e3 = 0;                                   // Sai so goc mat sut robot mong muon
// He so e3
float kpE3 = 2 ;
float kdE3 = 0 ;
float kiE3 = 0 ;
float pre_e3=0;
float ipartE3=0;
// He so e1
float kpE1 = 2.5 ;
float kdE1 = 0 ;
float kiE1 = 0 ;
float pre_e1=0;
float ipartE1=0;

// THONG SO CO BAN CUA ROBOT
uint8_t r = 58/2  ;                             // Ban kinh banh xe.
uint8_t R = 90    ;                             // Ban kinh khung xe
uint8_t dw = 13   ;                             // 1 nua do day banh xe

// DAI LUONG MONG MUON
float vR   = 500  ;                             // Toc do tham chieu can dat.
float wR          ;                             // Toc do goc
float vC          ;                             //= 200;
float RC   = 110  ;                             // Ban kinh xoay
float wC   = pi/3 ;                             //= vC/RC: Toc do goc                                                   
float dt   = 0.04 ;                             // Thoi gian lay mau 

// OUTPUT 
float w1 = 0, w2 = 0, w3 = 0;                   // Van toc goc 3 banh xe

// XET CASE
uint8_t step = 0  ;
uint8_t count=0   ;

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

void path(float x,float y,float theta,float x1, float y1)
{
    xB=x1;
    yB=y1;
    theta_start=thetabot;
    x_start=xbot;
    y_start=ybot;
    alpha23 = -theta_start + thetaball; // goc lech vector v so voi 23
    if      (alpha23>pi/3)      theta0 = 2*pi/3;
    else if (alpha23<-pi/3)     theta0 = -2*pi/3;
    else                        theta0 = 0;
}
void ball_moving(void)
{
    pre_vball=vrec;
    sumx=0;
    sumy=0;
    sumx2=0;
    sumy2=0;
    sumxy=0;
    vball=0;
    for (int i=0;i<3;i++)
    {
        pre_xball[3-i]=pre_xball[2-i];
        pre_yball[3-i]=pre_yball[2-i];
        vB[3-i]=vB[2-i];
    }
    pre_xball[0]=xball;
    pre_yball[0]=yball;
    vB[0]=vrec;
    for (int j=0;j<4;j++)
    {
        sumx = sumx + pre_xball[j];
        sumy = sumy + pre_yball[j];
        sumx2 = sumx2 + pre_xball[j]*pre_xball[j];
        sumy2 = sumy2 + pre_yball[j]*pre_yball[j];
        sumxy = sumxy + pre_xball[j]*pre_yball[j];
        vball = vball + vB[j];    
    }
    vball=vball/4;
    sxx = sumx2 - sumx*sumx/4;
    syy = sumy2 - sumy*sumy/4;
    sxy = sumxy - sumx*sumy/4;
    // y = a1*x+b1 => a1*x-y +b1=0
    a1 = sxy/sxx;
    b1 = sumy/4 - a1*sumx/4;
    if (counttheta<3) counttheta++;
    else
    {
        thetaball=atan2(sxy,sxx);
        counttheta=0;
    }
    printf("thetaball: %.2f\n",thetaball*180/pi);
    rate=sqrt((xball-xbot)*(xball-xbot)+(yball-ybot)*(yball-ybot));
    distance = (a1*xbot+b1-ybot)/sqrt(a1*a1+1);
}
void predict_time(void)
{
    deltat = 200/vball;    
    if (thetaball>0)    num=(xball-xbot)*cos(thetaball-pi)+(yball-ybot)*sin(thetaball-pi);
    else                num=(xball-xbot)*cos(thetaball+pi)+(yball-ybot)*sin(thetaball+pi);
    at=500*500-vball*vball;
    //bt=-2*(vR*deltat + vball*((xball-xbot)*cos(thetaball)+(yball-ybot)*sin(thetaball)));
    bt = -2*(vball*vball*deltat+vball*num);
    // ct=deltat*deltat*vR*vR - (xball-xbot)*(xball-xbot) - (yball-ybot)*(yball-ybot);
    ct=-2*vball*num*deltat-vball*vball*deltat*deltat - (xball-xbot)*(xball-xbot) - (yball-ybot)*(yball-ybot);
    tpredict= giaiPTBac2(at,bt,ct);
    // tcount=round(tpredict/dt);
    // deltat = 450/vball;    
    // if (thetaball>0)    num=(xball-xbot)*cos(thetaball-pi)+(yball-ybot)*sin(thetaball-pi);
    // else                num=(xball-xbot)*cos(thetaball+pi)+(yball-ybot)*sin(thetaball+pi);
    // at=500*500-300*300;
    // //bt=-2*(vR*deltat + vball*((xball-xbot)*cos(thetaball)+(yball-ybot)*sin(thetaball)));
    // bt = -2*(300*300*deltat+300*num);
    // // ct=deltat*deltat*vR*vR - (xball-xbot)*(xball-xbot) - (yball-ybot)*(yball-ybot);
    // ct=-2*300*num*deltat-300*300*deltat*deltat - (xball-xbot)*(xball-xbot) - (yball-ybot)*(yball-ybot);
    // tpredict= giaiPTBac2(at,bt,ct);
    // tcount=round(tpredict/dt);
}
void predict_pos(void)
{
    if (thetaball>0)
    {
        xt = xball+vball*tpredict*cos(thetaball-pi);
        yt = yball+vball*tpredict*sin(thetaball-pi);
    }
    else
    {
        xt = xball+vball*tpredict*cos(thetaball+pi);
        yt = yball+vball*tpredict*sin(thetaball+pi);
    }
    printf("xt: %.2f yt: %.2f\n",xt,yt);
    //tpredict = tcount*dt;
    //tcount--;
}

void calculate_error(void)
{
    pre_e3=e3;
    e1 = sqrt((xt-xbot)*(xt-xbot)+(yt-ybot)*(yt-ybot));
    e2 = -thetabot+atan2(yt-ybot,xt-xbot);                                                                        
    if (step==0 || step==1 || step==2) e3 = -thetabot-theta0+thetaball;//+atan2(yball-ybot,xball-xbot);
    else if (step==3)       e3 = -thetabot-theta0+atan2(0-ybot,1200-xbot);
    printf("e1: %.2f, e2 %.2f, e3: %.2f \n", e1, e2*180/pi, e3*180/pi);
}
void velbot(void)
{   
    // ipartE1=ipartE1+(e1+pre_e1)*dt;
    vR = kpE1*e1+kiE1*ipartE1+kdE1*(e1-pre_e1)/dt;
    if (e1>200)         
    {
        if (step==2)    vR=50;
        else            vR=500;
    }
    // ipartE3=ipartE3+(e3+pre_e3)*dt;
    w = kpE3*e3+kiE3*ipartE3+kdE3*(e3-pre_e3)/dt;
    if (w>pi/4)       w = pi/4;
    else if (w<-pi/4) w = -pi/4;
 
    vf=vR*cos(e2);
    vn=vR*sin(e2);
    printf("w: %f e1: %f\n", w,e1);
    ss << "Bot:"<< float(e1) << ":" << float(e2*180/pi) << ":" << float(e3*180/pi) << std::endl;
    error_msg.data = ss.str();   
    // printf("vel_pub: %s\n", vel_pub);
}

void position(void)
{
    //theta0=0;
    //printf("xstart: %.2f, ystart: %.2f, thetastart: %.2f\n", x_start, y_start, theta_start);
    if (abs(ybot)>500 || xbot<-50) step=5;
    if (step==1) 
    {
        // if (counttheta==0)
        // {
        predict_time();
        predict_pos();
        xerror=xt;
        yerror=yt;
        // }
    }
    if (step==1 || step==2 || step ==3)    calculate_error();  
    printf("e1: %.2f, e2 %.2f , e3: %2f , dis: %.2f\n",e1,e2,e3,distance);
    switch (step)
    {
        case 0:
        {
            vf=0;
            vn=0;
            w=0;
            if (vrec>80 && vrec<800)    
            {
                countb++;
                if (countb>=4)    
                {
                    predict_time();
                    if (tpredict==9999) step=5;
                    predict_pos();
                    xerror=xt;
                    yerror=yt;
                    path(xbot,ybot,thetabot,xball,yball);
                    calculate_error();
                    dir0=2;
                    step=1;
                }
                else step=0;
            }
            break;
        }
        case 1:
        {
            if (abs(e1)<= 50 && abs(e3)<pi/45) //&& abs(e3)>pi/190;distance < 30 (thetaball>0 && e3<=pi/60 && e3 || thetaball<0 && e3>=pi/60 && abs(e3)<=pi/90
            {
                // y = a1*x+b1 => a1*xp-yp +b1=0
                //(xbot-xp)+a1*(ybot-yt)=0
                //
                vf=0;
                vn=0;
                w=0;
                // if (rate>200)
                // {
                // xerror=(xbot+a1*ybot-a1*b1)/(1+a1*a1);
                // yerror=a1*xerror+b1;
                // }
                vsp=vball/3;
                step=5;
                //e3 = -thetabot-theta0+atan2(0-ybot,1200-xbot);
                break;
            }
            else
            {
                velbot();
                break;
            }
            //break;         
        }
        case 2:
        {
            if (rate<dis)
            {
            //B4: Giam toc 0.1s
            vf=-vsp*cos(theta0);
            vn=-vsp*sin(theta0);
            w=0;
            vel_calculate(&v1,&v2, &v3);
            vel_convert(vel_pub);
            serialPuts(fd, vel_pub);
            fflush(stdout);
            ros::Duration(1).sleep(); 
            step=3;
            e3 = -thetabot-theta0+atan2(0-ybot,1200-xbot);
            }
            else
            {
                velbot();
                vsp=vball/3;
                break;
            }
            //break;    
        }
        case 3:
        {
            if (abs(e3)>pi/180)
            {
                // if (abs(e3)>wC)
                // {
                w=kpE3*e3;
                if (w>2*wC)  w=kpE3*wC;
                else if (w<-2*wC)        w=-kpE3*wC;
                //else w=kpE3*e3;
                    // if (e3>0)
                    // {
                vf=w*RC*cos(theta0-pi/2);
                vn=w*RC*sin(theta0-pi/2);          
                break;
            }
            else    step=4;
        }
        case 4:
        {
            dir0=1;
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
            ros::Duration(0.4).sleep();
            
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
    count++;
    if (v1 < 0) dir1 = '1'; else dir1 = '0';
    if (v2 < 0) dir2 = '1'; else dir2 = '0'; 
    if (v3 < 0) dir3 = '1'; else dir3 = '0';

    v1 = abs(v1)*30/(r*pi);
    v2 = abs(v2)*30/(r*pi);
    v3 = abs(v3)*30/(r*pi);
    
    printf("w1: %.2f, w2: %.2f, w3: %.2f\n", v1, v2, v3);
    sprintf(vel_pub, "%c%d%c%d%c%d%d", dir1, int(round(v1+100)), dir2, int(round(v2+100)), dir3, int(round(v3+100)),dir0); 
        printf("vel_pub: %s\n", vel_pub);

}

void modesubcallback(const std_msgs::UInt8::ConstPtr& msg)
{
    if (msg->data == '9')
    {
        step=0;
        e2=0;
        e1=0;
        e3=0;
    }
}

void datasubcallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    w=0;
    vf=0;
    vn=0;
    
    if (countdir<=102) countdir++;
    if (countdir<5)         dir0=3;
    else if (countdir<10)   dir0=1;
    else dir0=2;
    // else if (countdir==100)  dir0=3;
    // else if (countdir==101)  dir0=2;
    // else                    dir0=1;
    vel_calculate(&v1, &v2, &v3);
    vel_convert(vel_pub);
    printf("\n");
    //printf( "Receive success\n");
    serialPuts(fd, vel_pub);
    fflush(stdout);
    // receive
    // do
    // {
	// 	vel_sub = serialGetchar(fd);
	// 	printf("%c",vel_sub);
	// 	fflush (stdout);
	// }
    // while(serialDataAvail(fd));
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
    while (1)
    {
        error_pub.publish(error_msg);
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    ros::spin();
}

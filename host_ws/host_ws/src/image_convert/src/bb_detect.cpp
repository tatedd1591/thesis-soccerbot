#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/calib3d/calib3d.hpp"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <vector>
#include <opencv2/aruco.hpp>
#include "std_msgs/String.h"
//#include "yaml-cpp/yaml.h"
//#include "bb_detect.h"

using namespace cv;
using namespace std;

struct Position
{
  float x;
  float y;
  float theta;
  float pre_x;
  float pre_y;
};
int16_t xmin = 115;
int16_t ymin = 16;
int16_t xmax = 681;
int16_t ymax = 583;

int iLowH = 30;
int iHighH = 71;

int iLowS = 47;
int iHighS = 254;

int iLowV = 65;
int iHighV = 255;

// double K[3][3] = {806.4156, 0, 410.9889, 0, 807.9814, 286.4361, 0,0, 1};
// double R[3][3] =  {-0.9998, 0.0104, -0.0154, -0.0104, -0.9999, -0.0022, -0.0154, -0.0021, 0.9999};
// double dist[5] =  {0.1440, -0.1194, 0, 0, -0.2915};
int32_t count_num;
vector<int> makerID;
vector<std::double_t> data;
vector<std::vector<cv::Point2f>> markerCorners;
aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
Ptr <cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
std::vector<cv::Vec4i> hiearchy;
vector<vector<cv::Point>> contours;
std_msgs::String pos_msg;
ros::Publisher pos_publisher;
float ball_vel;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void bot_detect(Mat img, Position* bot);
void ball_detect(Mat img, Position* ball);
void image_calib(Mat img, Mat* result);
void msg_generate(Position bot, Position ball);
double convert_coor(double height);
void ball_vel_cal(Position* object);

void ball_vel_cal(Position* object)
{
  ball_vel = sqrt(((object->pre_x - object->x)*(object->pre_x - object->x)) + ((object->pre_y - object->y)*(object->pre_y - object->y)))/0.033;
  object->pre_x = object->x;
  object->pre_y = object->y;
}
void bot_detect(Mat img, Position* bot)
{
  cv::aruco::detectMarkers(img, dictionary, markerCorners, makerID);
  if (makerID.size() > 0)
  {
    cv::aruco::drawDetectedMarkers(img, markerCorners, makerID);
    bot->x = (markerCorners[0][0].x + markerCorners[0][2].x)/2.0;
    bot->y = (markerCorners[0][0].y + markerCorners[0][2].y)/2.0; 
    bot->theta = atan2(-(markerCorners[0][1].y - markerCorners[0][0].y),(markerCorners[0][1].x - markerCorners[0][0].x))/3.14*180;
    // bot->x = (((bot->x-xmin)/((xmax-xmin)/1200.0)-600)*0.9449760610)+600;
    // bot->y = (((bot->y-ymin)/((ymax-ymin)/1200.0)-595)*0.942348668)+595;
    bot->x = (bot->x-xmin)/((xmax-xmin)/1200.0);
    bot->y = (bot->y-ymin)/((ymax-ymin)/1200.0);

    // if(bot->x > 800) bot->x -= 10;
    //printf("Bot position is: %.2f, %.2f\n", markerCorners[0][0].x, markerCorners[0][0].y);
  }
}
void ball_detect(Mat img, Position* ball)
{
  Mat dst;
  img.copyTo(dst);
  cv::cvtColor(dst, dst, cv::COLOR_BGR2HSV);
  inRange(dst, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), dst);
  erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

  dilate(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
	erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

  Moments oMoments = moments(dst);

  double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;

  if (dArea > 10000)
		{
			// Calculate the Centroid of the Object
			ball->x = dM10 / dArea;
			ball->y = dM01 / dArea;
    
      ball->x = (ball->x-xmin)/((xmax-xmin)/1200.0);
      ball->y = (ball->y-ymin)/((ymax-ymin)/1200.0);
      // if ((ball->x == 0) && (ball->y < 800) && (ball->y > 400)){
      //   float error = ball->y - 600;
      //   printf("GOALLLLLL SIUUUUUUUUU: %.2f\n", error);
      // }
			// Draw a Red Circle tracking the Object
			int R = sqrt((dArea / 255) / 3.14);
			if (ball->x >= 0 && ball->y >= 0)
			{
				circle(dst, Point(ball->x, ball->y), R, Scalar(0, 0, 255), 2);
      //  printf("Ball position: %.2f, %.2f\n", ball->x, ball->y);
			}
      // Find the Contour of the Object
			findContours(dst, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
			for (int i = 0; i< contours.size(); i++)
			{
				drawContours(img, contours, i, Scalar(0, 0, 255), 2);
			}
		}
  //imshow("detected", img);
}
void image_calib(Mat img, Mat* result)
{
  cv::Mat K = Mat(3,3, CV_64FC1);
  K.at<double>(0,0)= data[0];
  K.at<double>(0,1)= data[1];
  K.at<double>(0,2)= data[2];
  K.at<double>(1,0)= data[3];
  K.at<double>(1,1)= data[4];
  K.at<double>(1,2)= data[5];
  K.at<double>(2,0)= data[6];
  K.at<double>(2,1)= data[7];
  K.at<double>(2,2)= data[8];

  cv::Mat dist = Mat(5,1, CV_64FC1);
  dist.at<double>(0)= data[9];
  dist.at<double>(1)= data[10];
  dist.at<double>(2)= data[11];
  dist.at<double>(3)= data[12];
  dist.at<double>(4)= data[13];
  
 cv::undistort(img, *result, K, dist);
}
void msg_generate(Position bot, Position ball)
{
  std::stringstream ss;
  ss << "Host:"<< int(bot.x) << ":" << int(bot.y) << ":" << int(bot.theta) << ":" << int(ball.x) << ":" << int(ball.y) << ":" << int(ball_vel);
  pos_msg.data = ss.str();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Position bot_pos = {0}, ball_pos = {0};
    cv::Mat img_calibrated;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    image_calib(cv_ptr->image, &img_calibrated);
    bot_detect(img_calibrated, &bot_pos);
    // cout << "maker ID is: " << markerCorners[0] << endl;
    ball_detect(img_calibrated, &ball_pos);
    ball_vel_cal(&ball_pos);
    // flip(cv_ptr->image, cv_ptr->image, 1);
    //imshow("Original Image", cv_ptr->image);
    imshow("calibrated Image", img_calibrated);
    msg_generate(bot_pos, ball_pos);
    //if (ball_pos.x > 0 && ball_pos.y > 0)      
    printf("%s\n", pos_msg.data.c_str());
    cv::waitKey(3);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  nh_.getParam("/bb_detect/data", data);
  ros::Publisher pos_publisher = nh_.advertise<std_msgs::String>("/pos_pub/primitive", 1000);
  image_transport::Subscriber image_sub_ = it_.subscribe("image", 1, imageCallback);
  image_transport::Publisher image_pub_ = it_.advertise("/image_converter/output_video", 1);
  ros::Rate looprate(50);

  // namedWindow("Control");	
  // createTrackbar("LowH", "Control", &iLowH, 255);		//Hue (0 - 179)
	// createTrackbar("HighH", "Control", &iHighH, 255);

	// createTrackbar("LowS", "Control", &iLowS, 255);		//Saturation (0 - 255)
	// createTrackbar("HighS", "Control", &iHighS, 255);

	// createTrackbar("LowV", "Control", &iLowV, 255);		//Value (0 - 255)
	// createTrackbar("HighV", "Control", &iHighV, 255);

  //ImageConverter ic;
  while (ros::ok())
  {
    pos_publisher.publish(pos_msg);    
    looprate.sleep();
    ros::spinOnce();
  }
  ros::spin();
  return 0;
}
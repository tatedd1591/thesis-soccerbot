#ifndef DETECT_BB
#define DETECT_BB

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


using namespace cv;
using namespace std;

struct Position
{
  float x;
  float y;
  float theta;
};

// double K[3][3] = {806.4156, 0, 410.9889, 0, 807.9814, 286.4361, 0,0, 1};
// double R[3][3] =  {-0.9998, 0.0104, -0.0154, -0.0104, -0.9999, -0.0022, -0.0154, -0.0021, 0.9999};
// double dist[5] =  {0.1440, -0.1194, 0, 0, -0.2915};
int32_t count_num;
vector<int> makerID;
vector<std::vector<cv::Point2f>> markerCorners;
aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
Ptr <cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
std::vector<cv::Vec4i> hiearchy;
vector<vector<cv::Point>> contours;
std_msgs::String pos_msg;
ros::Publisher pos_publisher;
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void bot_detect(Mat img, Position* bot);
void ball_detect(Mat img, Position* ball);
void image_calib(Mat img, Mat* result);
void msg_generate(Position bot, Position ball);

#endif
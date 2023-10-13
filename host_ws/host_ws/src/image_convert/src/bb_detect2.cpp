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
#include <math.h>

using namespace cv;
using namespace std;

struct Position
{
  float x;
  float y;
  float theta;
  int pre_x;
  int pre_y;
  float vel;
};

int16_t xmin = 29;
int16_t ymin = 9;
int16_t xmax = 591;
int16_t ymax = 572;

int x_err1 = 42;
int x_err2 = 33;
int y_err1 = 34;
int y_err2 = 35;
double cam_height_x = 0;
double cam_height_y = 0;
double cam_x = 0;
double cam_y = 0;

int iLowH = 30;
int iHighH = 71;
int iLowS = 47;
int iHighS = 254;
int iLowV = 65;
int iHighV = 255;

Point p1(0, 290), p2(620, 290);
Point p3(310, 0), p4(310, 580);

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
Position bot_pos = {0}, ball_pos = {0};
int vel_count = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void bot_detect(Mat img, Position* bot);
void ball_detect(Mat img, Position* ball);
void image_calib(Mat img, Mat* result);
void msg_generate(Position bot, Position ball);
double convert_coor(double height);
void cam_height_detect(void);
double convert_coor(double obj_height, double img_cooor);

void cam_height_detect()
{
  cam_height_x = (1000*128.0 + 128.0*(x_err1+x_err2))/(x_err1 + x_err2);
  cam_height_y = (1000*128.0 + 128.0*(y_err1+y_err2))/(y_err1 + y_err2);
  cam_x = x_err1 * (cam_height_x - 128.0) / 128.0;
  cam_y = y_err1 * (cam_height_y - 128.0) / 128.0;
  //cout << cam_x << ":" << cam_y << endl;
}

double convert_coor(double obj_height, double x_value, double cam_height, double img_cooor)
{
  if (img_cooor <= x_value)
  {
    img_cooor = x_value - img_cooor;
    return x_value - (((cam_height-obj_height)*(img_cooor))/(cam_height));
  }
  else
  {
    img_cooor = img_cooor - x_value;
    return x_value + (((cam_height-obj_height)*(img_cooor))/(cam_height));
  }
}

void bot_detect(Mat img, Position* bot)
{
  cv::aruco::detectMarkers(img, dictionary, markerCorners, makerID);
  if (makerID.size() > 0) 
  {
    // int16_t bot_x_1 = markerCorners[0][0].x;
    // int16_t bot_x_2 = markerCorners[0][1].x;
    // int16_t bot_y_1 = markerCorners[0][0].y;
    // int16_t bot_y_2 = markerCorners[0][1].y;

    // bot_x_1 = (bot_x_1 - xmin)/((xmax - xmin)/1200.0);
    // bot_y_1 = (bot_y_1 - ymin)/((ymax - ymin)/1200.0);
    // bot_x_2 = (bot_x_2 - xmin)/((xmax - xmin)/1200.0);
    // bot_y_2 = (bot_y_2 - ymin)/((ymax - ymin)/1200.0);
    
    // bot_x_1 = convert_coor(118.0, cam_x, cam_height_x, bot_x_1);
    // bot_x_2 = convert_coor(118.0, cam_x, cam_height_x, bot_x_2);
    // bot_y_1 = convert_coor(118.0, cam_x, cam_height_x, bot_y_1);
    // bot_y_2 = convert_coor(118.0, cam_x, cam_height_x, bot_y_2);
    
    cv::aruco::drawDetectedMarkers(img, markerCorners, makerID);
    bot->x = (markerCorners[0][0].x + markerCorners[0][2].x)/2.0;
    bot->y = (markerCorners[0][0].y + markerCorners[0][2].y)/2.0; 
    // bot->theta = atan2(-(bot_y_2 - bot_y_1),(bot_x_2 - bot_x_1))/3.14*180;
    bot->theta = atan2(-(markerCorners[0][1].y - markerCorners[0][0].y),(markerCorners[0][1].x - markerCorners[0][0].x))/3.14*180;
    
    // printf("Bot position is: %.2f, %.2f\n", markerCorners[0][0].x, markerCorners[0][0].y);
    
    bot->x = (bot->x-xmin)/((xmax-xmin)/1200.0);
    bot->y = (bot->y-ymin)/((ymax-ymin)/1200.0);
    
    bot->x = convert_coor(128.0, cam_x, cam_height_x, bot->x);
    bot->y = convert_coor(128.0, cam_y, cam_height_y, bot->y);
    
    //printf("Bot position is: %.2f, %.2f, %.2f \n", bot->x, bot->y, bot->theta);
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
  // imshow("123", dst);
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
      ball->x = convert_coor(42.6, cam_x, cam_height_x, ball->x);
      ball->y = (ball->y-ymin)/((ymax-ymin)/1200.0);
      ball->y = convert_coor(42.6, cam_y, cam_height_y, ball->y);
      if ((vel_count%3) == 0)
      {
        ball->vel = (sqrt((ball->pre_x - ball->x)*(ball->pre_x - ball->x) + (ball->pre_y - ball->y)*(ball->pre_y - ball->y)))*(1/0.12);
        ball->pre_x = ball->x;
        ball->pre_y = ball->y;
        vel_count = 0;
      }
      
      //cout << (ball->pre_x) << " : " << (ball->pre_y) << " : " << (ball->x) << " : " << (ball->y) << " : " << ball->vel << endl;
      vel_count += 1;


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
  // //imshow("detected", img);
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
  ss << "Host:"<< int(bot.x) << ":" << int(bot.y) << ":" << int(bot.theta) << ":" << int(ball.x) << ":" << int(ball.y) << ":" << int(ball.vel);
  count_num = count_num + 1;
  pos_msg.data = ss.str();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img_calibrated, cropped_image, dst;
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
    cropped_image = img_calibrated(Range(10,590), Range(90,710));
    // Canny(cropped_image, dst, 230, 250, 3);
	  // imshow("DST", dst);
    bot_detect(cropped_image, &bot_pos);
    // // cout << "maker ID is: " << markerCorners[0] << endl;
    ball_detect(cropped_image, &ball_pos);
    // flip(cv_ptr->image, cv_ptr->image, 1);
    //imshow("Original Image", cv_ptr->image);
    line(cropped_image, p1, p2, 1);
    line(cropped_image, p3, p4, 1);
    msg_generate(bot_pos, ball_pos);
    //if (ball_pos.x > 0 && ball_pos.y > 0)
    if ((ball_pos.x > -7) && (ball_pos.x < 7)){
        float error = ball_pos.y - 600;
        //printf("Error Kicking: %.2f   ", error);
        //printf("%s   ", pos_msg.data.c_str());
        printf("Robot pos: (%d,%d,%d)    \n", 1200-int(bot_pos.x), int(bot_pos.y)-600, int(bot_pos.theta));
        printf("Ball pos: (%d,%d,%d)\n", 1200-int(ball_pos.x), int(ball_pos.y)-600, int(ball_pos.theta));
      }
      else 
      {
        //printf("Error Kicking: 000  ");
        //printf("%s   ", pos_msg.data.c_str());
        printf("Robot pos: (%d,%d,%d)    \n", 1200-int(bot_pos.x), int(bot_pos.y)-600, int(bot_pos.theta));
        printf("Ball pos: (%d,%d,%d)\n", 1200-int(ball_pos.x), int(ball_pos.y)-600, int(ball_pos.theta));
      }      
 
    string a = "Bot: (" + to_string(1200-int(bot_pos.x)) + ", " + to_string(int(bot_pos.y)-600) + "," + to_string(int(bot_pos.theta))+ ")";
    string b = "Ball: (" + to_string(1200-int(ball_pos.x)) + ", " + to_string(int(ball_pos.y)-600) + ")";
    string c = "Ball velocity: " + to_string(ball_pos.vel);
    putText(cropped_image, a, cv::Point(40,30),cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(0,0,0), 1.5, false);
    putText(cropped_image, b, cv::Point(40,60),cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(0,0,0), 1.5, false);
    //putText(cropped_image, c, cv::Point(40,90),cv::FONT_HERSHEY_DUPLEX, 0.65, cv::Scalar(255,255,255), 1.5, false);
    imshow("Image", cropped_image);

    cv::waitKey(3);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter_v2");
  ros::NodeHandle nh_;
  cam_height_detect();
  //printf("Camera_height: %.2f\n", cam_height);
  image_transport::ImageTransport it_(nh_);
  nh_.getParam("/bb_detect/data", data);
  ros::Publisher pos_publisher = nh_.advertise<std_msgs::String>("/pos_pub/primitive", 1000);
  image_transport::Subscriber image_sub_ = it_.subscribe("image", 1, imageCallback);
  image_transport::Publisher image_pub_ = it_.advertise("/image_converter/output_video", 1);
  ros::Rate looprate(25);

  // namedWindow("Control");	
  // createTrackbar("LowH", "Control", &iLowH, 255);		//Hue (0 - 179)
	// createTrackbar("HighH", "Control", &iHighH, 255);

	// createTrackbar("LowS", "Control", &iLowS, 255);		//Saturation (0 - 255)
	// createTrackbar("HighS", "Control", &iHighS, 255);

	// createTrackbar("LowV", "Control", &iLowV, 255);		//Value (0 - 255)
	// createTrackbar("HighV", "Control", &iHighV, 255);
  while (ros::ok())
  {
    pos_publisher.publish(pos_msg);    
    looprate.sleep();
    ros::spinOnce();
  }
  ros::spin();
  return 0;
}
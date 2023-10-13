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

int main(int argc, char** argv)
{
    std::vector<std::double_t> data;
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "test_node");
	//create a node handle: it is reference assigned to a new node
	ros::NodeHandle n;
    n.getParam("/test_node/data", data);
	//create a publisher with a topic "chatter" that will send a String message

    // Declare the output variables
    Mat dst, cdst, cdstP;
    const char* default_file = "/home/jacky/Documents/Jacky/Calib/WIN_20230425_12_19_58_Pro.jpg";
    // Loads an image
    Mat src = imread("/home/jacky/Documents/Jacky/Calib/2023-05-04-153323.jpg", IMREAD_GRAYSCALE );
    // Check if image is loaded fine
    if(src.empty()){
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", default_file);
        return -1;
    }
    Mat calibed;
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
  
    undistort(src, calibed, K, dist);
    imshow("cropped", calibed);

    Mat cropped_image = calibed(Range(10,590), Range(60,700));
//	imshow("Source", cropped_image);

    //Edge detection
    Canny(cropped_image, dst, 200, 220, 3);

    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdst, COLOR_GRAY2BGR);
    cdstP = cdst.clone();
    // Standard Hough Line Transform
    vector<Vec2f> lines; // will hold the results of the detection
    HoughLines(dst, lines, 1, CV_PI/180, 180, 0, 0 ); // runs the actual detection
    // Draw the lines
    int16_t xmin, xmax, ymin, ymax;
    int16_t temp1, temp2;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, Scalar(0,0,255), 1, LINE_AA);
        line( cropped_image, pt1, pt2, Scalar(0,0,255), 1, LINE_AA);
        cout << i << " " << pt1.x << " " << pt1.y << " " << pt2.x << " " << pt2.y << endl;
    }

    // //Show results
    //cvtColor(cropped_image, cropped_image, COLOR_GRAY2BGR);
    imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
    imshow("DST", cropped_image);
    
    // Wait and Exit
    waitKey(0);
    return 0;
}

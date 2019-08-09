#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv_bridge;
using namespace cv;

cv::Mat cameraMatrix;
cv::Mat distCoeffs;

void callback(const sensor_msgs::ImageConstPtr& msg)
{
  CvImagePtr imgPtr = toCvCopy(msg, "bgr8");
  cv::Mat img = imgPtr->image;
  cv::undistort(img,img,cameraMatrix,distCoeffs);
  cv::imshow("view", img);
  cv::waitKey(30);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  cameraMatrix = Mat::zeros(3, 3, CV_64F);
  distCoeffs = Mat::zeros(5, 1, CV_64F);

  double fx = 3388.49;
  double fy = 3390.57;
  double cx = 2096.43;
  double cy = 1566.23;
  double skew = 1.11273;

  double k1 = 0.17352;
  double k2 = -0.484226;
  double k3 = 0.344761;
  double p1 = 0.00075256;
  double p2 = -0.000269617;

  cameraMatrix.at<double>(0,0)=fx;
  cameraMatrix.at<double>(1,1)=fy;
  cameraMatrix.at<double>(0,1)=skew;
  cameraMatrix.at<double>(0,2)=cx;
  cameraMatrix.at<double>(1,2)=cy;
  cameraMatrix.at<double>(2,2)=1;

  //Set the distortion matrix
  distCoeffs.at<double>(0,0)=k1;
  distCoeffs.at<double>(1,0)=k2;
  distCoeffs.at<double>(2,0)=p1;
  distCoeffs.at<double>(3,0)=p2;
  distCoeffs.at<double>(4,0)=k3;

  cout << cameraMatrix << endl;
  cout << distCoeffs << endl;

  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, callback);
  ros::spin();
  cv::destroyWindow("view");
}

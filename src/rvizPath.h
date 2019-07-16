#ifndef RVIZPATH_H
#define RVIZPATH_H

#define DEFAULT_NUM_OF_POSE (500)

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <common_def.h>


class RVIZPath
{
private:
  ros::Publisher publisher;
  Mat4x4 pose;
  int numOfPose;
  float quaternion[4];
  float point[3];

public:

  RVIZPath(ros::NodeHandle& nh, string topic_name, int bufferCount=10, int maxNumOfPose=-1)
  {
    publisher = nh.advertise<visualization_msgs::Marker>(topic_name, bufferCount);
    if(maxNumOfPose==-1)
    {
      numOfPose = DEFAULT_NUM_OF_POSE;
    }else
    {
      numOfPose = maxNumOfPose;
    }
  }
  ~RVIZPath();

  pubPath();

};//class RVIZPath



#endif // RVIZPATH_H

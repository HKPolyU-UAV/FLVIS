#ifndef RVIZPATH_H
#define RVIZPATH_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <common_def.h>


class RVIZPath
{
private:

  ros::NodeHandle n;
  ros::Publisher publisher;
  Mat4x4 pose;


public:

  RVIZPath(ros::NodeHandle image_width, int image_height, int boundaryBoxSize=5)
  {

  }
  ~RVIZPath();

  pubPath();

};//class RVIZPath



#endif // RVIZPATH_H

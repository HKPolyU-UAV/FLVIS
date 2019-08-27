#ifndef RVIZ_TF_H
#define RVIZ_TF_H

#include <include/common.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class RVIZTF
{

public:
    tf::TransformBroadcaster br;
    tf::Transform transform;

    RVIZTF();
    void pubTFT_c_w(const SE3 T_c_w, const ros::Time stamp=ros::Time::now());


};

#endif // RVIZ_TF_H

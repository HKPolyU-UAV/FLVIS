#ifndef RVIZ_POSE_H
#define RVIZ_POSE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <include/common.h>

class RVIZPose
{
private:

    ros::Publisher pose_pub;
    geometry_msgs::PoseStamped pose;

public:
    RVIZPose();
    ~RVIZPose();
    RVIZPose(ros::NodeHandle& nh,
             string topicName, string frameId,
             int bufferSize=2);
    void pubPose(const Quaterniond q,const Vec3 t, const ros::Time stamp=ros::Time::now());




};

#endif // RVIZ_POSE_H

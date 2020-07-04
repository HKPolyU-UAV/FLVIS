#ifndef RVIZ_ODOM_H
#define RVIZ_ODOM_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <include/common.h>

class RVIZOdom
{
private:

    ros::Publisher odom_pub;
    nav_msgs::Odometry odom;

public:
    RVIZOdom();
    ~RVIZOdom();
    RVIZOdom(ros::NodeHandle& nh,
             string topicName, string frameId,
             int bufferSize=2);
    void pubOdom(const Quaterniond q,const Vec3 t, const Vec3 v, const ros::Time stamp=ros::Time::now());




};

#endif // RVIZ_POSE_H

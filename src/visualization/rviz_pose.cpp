#include "include/rviz_pose.h"

RVIZPose::RVIZPose()
{

}

RVIZPose::RVIZPose(ros::NodeHandle& nh,
                   string topicName, string frameId,
                   int bufferSize)
{
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topicName, bufferSize);
    this->pose.header.frame_id = frameId;
}

void RVIZPose::pubPose(const Quaterniond q, const Vec3 t, const ros::Time stamp)
{
    pose.header.stamp = stamp;
    pose.pose.position.x=t(0);
    pose.pose.position.y=t(1);
    pose.pose.position.z=t(2);
    pose.pose.orientation.w=q.w();
    pose.pose.orientation.x=q.x();
    pose.pose.orientation.y=q.y();
    pose.pose.orientation.z=q.z();
    pose_pub.publish(pose);
}

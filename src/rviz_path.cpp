#include <include/rviz_path.h>


RVIZPath::RVIZPath(ros::NodeHandle& nh, string topic_name, int bufferCount, int maxNumOfPose)
{
  path_pub = nh.advertise<nav_msgs::Path>(topic_name, bufferCount);
  if(maxNumOfPose==-1)
  {
    numOfPose = DEFAULT_NUM_OF_POSE;
  }else
  {
    numOfPose = maxNumOfPose;
  }
  path.header.frame_id = "map";
}

void RVIZPath::pubPathT_c_w(const SE3 T_c_w, const ros::Time stamp)
{
  pubPathT_w_c(T_c_w.inverse(),stamp);
}

void RVIZPath::pubPathT_w_c(const SE3 T_w_c, const ros::Time stamp)
{
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id="map";
  poseStamped.header.stamp = ros::Time::now();

  Quaterniond q = T_w_c.so3().unit_quaternion();
  Vec3        t = T_w_c.translation();

  poseStamped.pose.orientation.w = q.w();
  poseStamped.pose.orientation.x = q.x();
  poseStamped.pose.orientation.y = q.y();
  poseStamped.pose.orientation.z = q.z();
  poseStamped.pose.position.x = t[0];
  poseStamped.pose.position.y = t[1];
  poseStamped.pose.position.z = t[2];

  path.header.stamp = stamp;
  path.poses.push_back(poseStamped);

  if(path.poses.size()>=numOfPose)
  {
    path.poses.erase(path.poses.begin());
  }

  path_pub.publish(path);
}

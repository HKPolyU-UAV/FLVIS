#ifndef RVIZFRAME_H
#define RVIZFRAME_H

#include <ros/ros.h>
#include <include/common.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>



class RVIZFrame
{
private:
  ros::Publisher pose_pub;
  ros::Publisher marker_pub;
  string frame_id_pose;
  string frame_id_pts;
  float f;

  void pubFramePtsPoseT_w_c(const vector<Vec3>& pts3d,
                            const SE3 T_w_c=SE3(),
                            const ros::Time stamp=ros::Time::now());

  void pubFramePoseT_w_c(const SE3 T_w_c,
                         const ros::Time stamp=ros::Time::now());
public:

  RVIZFrame(ros::NodeHandle& nh,
            string poseTopicName, string frameIDPose,
            string ptsTopicName,  string frameIDPts,
            int bufferPose=10,
            int bufferPts=10);

  ~RVIZFrame();

  void pubFramePtsPoseT_c_w(const vector<Vec3>& pts3d,
                            const SE3 T_c_w=SE3(),
                            const ros::Time stamp=ros::Time::now());
  void pubFramePoseT_c_w(const SE3 T_c_w,
                         const ros::Time stamp=ros::Time::now());

};

#endif // RVIZFRAME_H

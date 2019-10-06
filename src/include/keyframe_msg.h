#ifndef KEYFRAME_MSG_H
#define KEYFRAME_MSG_H
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <vo_nodelet/KeyFrame.h>
#include <include/common.h>
#include <camera_frame.h>




class KeyFrameMsg
{
    ros::Publisher kf_pub;
public:
    KeyFrameMsg();
    KeyFrameMsg(ros::NodeHandle& nh, string topic_name, int buffersize=2);
    void pub(CameraFrame& frame, ros::Time stamp=ros::Time::now());
    static void unpack(vo_nodelet::KeyFrameConstPtr kf_const_ptr,
                       cv::Mat& img,
                       vector<uint64_t>& lm_id,
                       vector<Vec2>& lm_2d,
                       vector<Vec3>& lm_3d,
                       vector<Mat> & lm_descriptors,
                       SE3& pose);
};

#endif // KEYFRAME_MSG_H

#ifndef KEYFRAME_MSG_H
#define KEYFRAME_MSG_H
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <flvis/KeyFrame.h>
#include <include/common.h>
#include <include/camera_frame.h>


struct KeyFrameStruct {
    cv::Mat         img;
    cv::Mat         d_img;
    int64_t         frame_id;
    int             lm_count;
    vector<int64_t> lm_id;
    vector<Vec2>    lm_2d;
    vector<Vec3>    lm_3d;
    vector<cv::Mat>     lm_descriptor;
    SE3             T_c_w;
};

class KeyFrameMsg
{
    ros::Publisher kf_pub;
public:
    KeyFrameMsg();
    KeyFrameMsg(ros::NodeHandle& nh, string topic_name, int buffersize=2);
    void pub(CameraFrame& frame, ros::Time stamp=ros::Time::now());
    static void unpack(flvis::KeyFrameConstPtr kf_const_ptr,
                       int64_t         &frame_id,
                       cv::Mat             &img,
                       cv::Mat             &d_img,
                       int             &lm_count,
                       vector<int64_t> &lm_id,
                       vector<Vec2>    &lm_2d,
                       vector<Vec3>    &lm_3d,
                       vector<cv::Mat>     &lm_descriptors,
                       SE3             &T_c_w,
                       ros::Time       &T);
};

#endif // KEYFRAME_MSG_H

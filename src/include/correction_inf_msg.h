#ifndef CORRECTIONINFMSG_H
#define CORRECTIONINFMSG_H

#include <ros/ros.h>

#include <vo_nodelet/CorrectionInf.h>
#include <include/common.h>
#include <include/camera_frame.h>

struct CorrectionInfStruct {
    int64_t         frame_id;
    SE3             T_c_w;
    int             lm_count;
    vector<int64_t> lm_id;
    vector<Vec3>    lm_3d;
    int             lm_outlier_count;
    vector<int64_t> lm_outlier_id;
};

class CorrectionInfMsg
{
    ros::Publisher correction_inf_pub;
public:
    CorrectionInfMsg();
    CorrectionInfMsg(ros::NodeHandle& nh, string topic_name, int buffersize=1);

    void pub(const int64_t         &frame_id_in,
             const SE3             &T_c_w_in,
             const int             &lm_count_in,
             const vector<int64_t> &lm_id_in,
             const vector<Vec3>    &lm_3d_in,
             const int             &lm_outlier_count_in,
             const vector<int64_t> &lm_outlier_id_in,
             ros::Time             stamp=ros::Time::now());

    static void unpack(vo_nodelet::CorrectionInfConstPtr c_inf_ptr,
                       int64_t                           &frame_id_out,
                       SE3                               &T_c_w_out,
                       int                               &lm_count_out,
                       vector<int64_t>                   &lm_id_out,
                       vector<Vec3>                      &lm_3d_out,
                       int                               &lm_outlier_count_out,
                       vector<int64_t>                   &lm_outlier_id_out);
};

#endif // CORRECTIONINFMSG_H

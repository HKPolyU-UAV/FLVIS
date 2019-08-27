#include "landmark_in_frame.h"

LandMarkInFrame::LandMarkInFrame(const Mat descriptor, const Vec2 pt2d, const Vec3 pt3d_c, const bool has_3d_inf, const SE3 T_c_w)
    :LandMark()
{
    lm_descriptor = descriptor;
    lm_1st_obs_2d = lm_2d = pt2d;
    lm_frame_pose = lm_1st_obs_frame_pose =T_c_w;
    lm_tracking_state = LM_TRACKING_INLIER;

    if(has_3d_inf){
        lm_3d_w = DepthCamera::camera2worldT_c_w(pt3d_c,T_c_w);
        lm_has_3d = true;
        lm_ob_dir = lm_3d_w - T_c_w.inverse().translation();
        double norm=sqrt(pow(lm_ob_dir[0],2)+pow(lm_ob_dir[1],2)+pow(lm_ob_dir[2],2));
        lm_ob_dir = lm_ob_dir/norm;
    }else{
        lm_3d_w = Vec3(0,0,0);
        lm_ob_dir = Vec3(0,0,0);
        lm_has_3d = false;
    }
}

bool LandMarkInFrame::hasDepthInf(void)
{
    return lm_has_3d;
}

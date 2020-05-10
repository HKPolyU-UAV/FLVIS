#include "include/landmark.h"

static int64_t id_index=100;

LandMark::LandMark()
{
    lm_id=id_index;
    id_index++;
}

LandMark::LandMark(const Vec3 pt3d_w_in)
{
    lm_id=id_index;
    id_index++;
    lm_3d_w = pt3d_w_in;
}

LandMarkInFrame::LandMarkInFrame(const Vec2 pt2d, const Vec3 pt3d_c, const bool has_3d_inf, const SE3 T_c_w)
    :LandMark()
{
    lm_1st_obs_2d = lm_2d = pt2d;
    lm_1st_obs_frame_pose =T_c_w;
    is_tracking_inlier = true;
    is_belong_to_kf = false;

    if(has_3d_inf){
        lm_3d_w = DepthCamera::camera2worldT_c_w(pt3d_c,T_c_w);
        has_3d = true;
    }else{
        lm_3d_w = Vec3(0,0,0);
        has_3d = false;
    }
}

bool LandMarkInFrame::hasDepthInf(void)
{
    return has_3d;
}

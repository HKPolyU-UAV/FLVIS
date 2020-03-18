#ifndef LANDMARK_IN_FRAME_H
#define LANDMARK_IN_FRAME_H

#include "include/depth_camera.h"
#include "include/common.h"

#define LMSTATE_NO_DEPTH  (0)
#define LMSTATE_NORMAL    (1)
#define LM_TRACKING_INLIER  (0)
#define LM_TRACKING_OUTLIER (1)

class LandMark
{
public:
  int64_t lm_id;
  Vec3    lm_3d_w;
  cv::Mat lm_descriptor;
  Vec3    lm_ob_dir;
  //No depth Information
  //Normal
  //Depth Converged
  //Error
  unsigned char lmState;
  LandMark();
  LandMark(const cv::Mat descriptor_in,const Vec3 pt3d_w_in,Vec3 ob_dir_in = Vec3(0,0,0));
};

class LandMarkInFrame : public LandMark
{
public:
    Vec2 lm_2d;
    Vec3 lm_3d_c;      //land mark 3d in camera frame
    SE3  lm_frame_pose;//pose of the frame
    bool lm_has_3d;
    unsigned char lm_tracking_state;
    Vec2 lm_1st_obs_2d;
    SE3  lm_1st_obs_frame_pose;//camera pose of first observation(for triangulation only)
    LandMarkInFrame(const cv::Mat descriptor,
                    const Vec2 pt2d,
                    const Vec3 pt3d_c,
                    const bool has_3d_inf,
                    const SE3 T_c_w);
    bool hasDepthInf(void);
};


#endif // LANDMARK_IN_FRAME_H

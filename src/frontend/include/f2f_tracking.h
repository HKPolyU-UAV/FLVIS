#ifndef F2F_TRACKING
#define F2F_TRACKING

#include "include/feature_dem.h"
#include "include/lkorb_tracking.h"
#include "include/camera_frame.h"
#include "include/vi_motion.h"
#include "include/keyframe_msg.h"
#include "include/correction_inf_msg.h"
#include "include/optimize_in_frame.h"

using namespace std::chrono;
using namespace cv;

enum TRACKINGSTATE{UnInit,
                   Tracking,
                   TrackingFail};

struct ID_POSE {
  int    frame_id;
  SE3    T_c_w;
};

class F2FTracking
{
public:
  enum TYPEOFCAMERA cam_type;
  //Modules
  FeatureDEM         *feature_dem;
  LKORBTracking      *lkorb_tracker;
  VIMOTION           *vimotion;
  DepthCamera         d_camera;
  float iir_ratio;
  float range;
  bool  enable_dummy;

  //states:
  bool has_imu;
  bool has_localmap_feedback;
  int  frameCount;
  enum TRACKINGSTATE   vo_tracking_state;
  int  skip_n_imgs;
  bool need_equal_hist;

  //varialbe
  CorrectionInfStruct correction_inf;
  SE3 T_c_w_last_keyframe;
  deque<ID_POSE> pose_records;
  CameraFrame::Ptr curr_frame,last_frame;

  void init(const DepthCamera dc_in,
            const SE3 T_i_c0_in,
            const Vec6 feature_para,
            const Vec6 vi_para,
            const Vec3 dr_para,
            const int skip_first_n_imgs_in,
            const bool need_equal_hist_in
            );

  void correction_feed(const double time, const CorrectionInfStruct corr);

  void imu_feed(const double time,
                const Vec3 acc,
                const Vec3 gyro,
                Quaterniond& q_w_i,
                Vec3& pos_w_i,
                Vec3& vel_w_i);

  void image_feed(const double time,
                  const cv::Mat img0_in,
                  const cv::Mat img1_in,
                  bool &new_keyframe,
                  bool &reset_cmd);

private:
  bool init_frame(void);

};//class F2FTracking

#endif

#ifndef F2F_TRACKING
#define F2F_TRACKING

#include "include/feature_dem.h"
#include "include/lkorb_tracking.h"
#include "include/camera_frame.h"
#include "include/vi_motion.h"
#include "include/keyframe_msg.h"
#include "include/correction_inf_msg.h"
#include "include/optimize_in_frame.h"

enum TRACKINGSTATE{UnInit, Tracking, TrackingFail};
enum TYPEOFCAMERA{DEPTH_D435I,STEREO_EuRoC_MAV};
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

    //states:
    bool has_imu;
    bool has_localmap_feedback;
    int  frameCount;
    enum TRACKINGSTATE   vo_tracking_state;

    //varialbe
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    CorrectionInfStruct correction_inf;
    SE3 T_c_w_last_keyframe;
    deque<ID_POSE> pose_records;
    CameraFrame::Ptr curr_frame,last_frame;

    void imu_feed(const double time,
                  const Vec3 acc,
                  const Vec3 gyro,
                  Quaterniond& q_w_i,
                  Vec3& pos_w_i,
                  Vec3& vel_w_i);

    void image_feed(const double time,
                    const cv::Mat img0,
                    const cv::Mat img1,
                    bool &new_keyframe,
                    bool &reset_cmd);

    void correction_feed(const double time, const CorrectionInfStruct corr);

    void init(const int w, const int h,
              const double cam0_fx_in, const double cam0_fY_in,
              const double cam0_cx_in, const double cam0_cy_in,
              const SE3 T_i_c0_in,
              const TYPEOFCAMERA cam_type_in=DEPTH_D435I,
              const double cam_scale_in=1000.1,
              const double cam1_fx_in=0.0, const double cam1_fy_in=0.0,
              const double cam1_cx_in=0.0, const double cam1_cy_in=0.0,
              const SE3 T_cam0_cam1=SE3());


};//class F2FTracking

#endif

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

    //states:
    bool has_imu;
    bool has_localmap_feedback;
    int  frameCount;
    enum TRACKINGSTATE   vo_tracking_state;

    //varialbe
    cv::Mat K0,D0,K1,D1;
    cv::Mat K0_rect;//cam0 rectified cameraMatrix;
    cv::Mat D0_rect;//cam0 rectified distCoeffs;
    cv::Mat K1_rect;//cam1 rectified cameraMatrix;
    cv::Mat D1_rect;//cam1 rectified distCoeffs;
    cv::Mat c0_RM[2];
    cv::Mat c1_RM[2];

    CorrectionInfStruct correction_inf;
    SE3 T_c_w_last_keyframe;
    deque<ID_POSE> pose_records;
    CameraFrame::Ptr curr_frame,last_frame;

    void init(const int w, const int h,
              const Mat c0_cameraMatrix_in,
              const Mat c0_distCoeffs_in,
              const SE3 T_i_c0_in,
              const Vec6 feature_para,
              const Vec4 vi_para=Vec4(0,0,0,0),
              const TYPEOFCAMERA cam_type_in=DEPTH_D435,
              const double cam_scale_in=1000.0,
              const Mat c1_cameraMatrix_in=Mat1d(3, 3),
              const Mat c1_distCoeffs_in=Mat1d(4, 1),
              const SE3 T_c0_c1=SE3());

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

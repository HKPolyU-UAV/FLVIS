#ifndef TRACKING_H
#define TRACKING_H

//Lucas-Kanade tracking with ORB feature verify

#include "camera_frame.h"

class LKORBTracking
{
    int width,height;
public:
    DepthCamera  d_camera;
    LKORBTracking(int width_in,int height_in);
    bool tracking(CameraFrame &from,
                  CameraFrame &to,
                  SE3 T_c_w_guess,
                  bool use_guess,
                  vector<cv::Point2f>& lm2d_from,
                  vector<cv::Point2f>& lm2d_to,
                  vector<cv::Point2f>& outlier);
};

#endif // F2FTRACKING_H

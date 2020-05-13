#ifndef F2FTRACKING_H
#define F2FTRACKING_H

//Lucas-Kanade tracking with ORB feature verify

#include "camera_frame.h"

class LKORBTracking
{
    int width,height;
public:
    LKORBTracking(int width_in,int height_in);
    bool tracking(CameraFrame &from,
                  CameraFrame &to,
                  vector<cv::Point2f>& lm2d_from,
                  vector<cv::Point2f>& lm2d_to,
                  vector<cv::Point2f>& outlier);
};

#endif // F2FTRACKING_H

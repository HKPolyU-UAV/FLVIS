#ifndef F2FTRACKING_H
#define F2FTRACKING_H

#include "camera_frame.h"

class F2FTracking
{
    int width,height;
public:
    F2FTracking(int width_in,int height_in);
    bool tracking(CameraFrame &from,
                  CameraFrame &to,
                  vector<Vec2>& lm2d_from,
                  vector<Vec2>& lm2d_to,
                  vector<Vec2>& outlier);
};

#endif // F2FTRACKING_H

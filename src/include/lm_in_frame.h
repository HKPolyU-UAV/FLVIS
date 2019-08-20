#ifndef LANDMARKINFRAME_H
#define LANDMARKINFRAME_H

#include "include/landmark.h"

class LandMarkInFrame : public LandMark
{
public:

  Vec2 lmPt2d;

  LandMarkInFrame(Vec2 pt2d, Vec3 pt3d, Mat descriptor, unsigned char has3dInf)
    :LandMark(pt3d,descriptor,has3dInf)
  {
    lmPt2d = pt2d;
  }

};

#endif // LANDMARKINFRAME_H

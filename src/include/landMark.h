#ifndef LandMark_H
#define LandMark_H

#include <include/common.h>

#define LMSTATE_NO_DEPTH  (0)
#define LMSTATE_NORMAL    (1)
#define LMSTATE_CONVERGED (2)
#define LMSTATE_ERROR     (3)

class LandMark
{

public:

  Vec3 lmPt3d;
  Mat  lmDescriptor;
  int  lmObsTimes;


  //No depth Information
  //Normal
  //Depth Converged
  //Error
  unsigned char lmState;

  LandMark(const Vec3 pt_in,const Mat descriptor_in, const unsigned char has3DInf_in);
  unsigned char hasDepthInf();

};

#endif // LandMark_H

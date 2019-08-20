#include <include/landmark.h>

//#define LMSTATE_NO_DEPTH  (0)
//#define LMSTATE_NORMAL    (1)
//#define LMSTATE_CONVERGED (2)
//#define LMSTATE_ERROR     (3)

LandMark::LandMark(const Vec3 pt_in,const Mat descriptor_in, const unsigned char has3DInf_in)
{

  lmPt3d = pt_in;
  lmDescriptor = descriptor_in;
  if(has3DInf_in)
  {
    lmState=LMSTATE_NORMAL;
  }
  else
  {
    lmState=LMSTATE_NO_DEPTH;
  }
  lmObsTimes = 1;
}

unsigned char LandMark::hasDepthInf()
{
  if(lmState==LMSTATE_NORMAL || lmState==LMSTATE_CONVERGED)
  {return 1;}
  else
  {return 0;}
}

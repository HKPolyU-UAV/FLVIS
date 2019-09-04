#ifndef LANDMARK_H
#define LANDMARK_H

#include <include/common.h>
#include <include/unique_id.h>

#define LMSTATE_NO_DEPTH  (0)
#define LMSTATE_NORMAL    (1)

class LandMark
{

public:

  uint64_t lm_id;
  //long lm_id
  Vec3 lm_3d_w;
  Mat  lm_descriptor;
  Vec3 lm_ob_dir;

  //No depth Information
  //Normal
  //Depth Converged
  //Error
  unsigned char lmState;

  LandMark();
  LandMark(const Mat descriptor_in,const Vec3 pt3d_w_in,Vec3 ob_dir_in = Vec3(0,0,0));

};

#endif // LandMark_H

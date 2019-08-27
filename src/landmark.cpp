#include <include/landmark.h>

LandMark::LandMark()
{
    lm_id=uniqueID::getID();
}

LandMark::LandMark(const Mat descriptor_in,const Vec3 pt3d_w_in,Vec3 ob_dir_in)
{
  lm_id=uniqueID::getID();
  lm_descriptor = descriptor_in;
  lm_3d_w = pt3d_w_in;
  lm_ob_dir = ob_dir_in;
}


#include <include/landmark.h>

static int64_t id_index=100;

LandMark::LandMark()
{
    lm_id=id_index;
    id_index++;
}

LandMark::LandMark(const cv::Mat descriptor_in,const Vec3 pt3d_w_in,Vec3 ob_dir_in)
{
    lm_id=id_index;
    id_index++;
    lm_descriptor = descriptor_in;
    lm_3d_w = pt3d_w_in;
    lm_ob_dir = ob_dir_in;
}


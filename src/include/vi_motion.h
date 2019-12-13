#ifndef VI_MOTION_H
#define VI_MOTION_H

#include <include/common.h>
#include <include/imu_state.h>

struct MOTION_STATE{
    Vec3 pos;
    Vec3 vel;
    Vec4 rot_q;

    IMUSTATE imu_read;
};

class VIMOTION
{
public:
  Vec3 acc_bias;
  Vec3 gyro_bias;
  std::vector<MOTION_STATE> states;
  bool uninit;

  VIMOTION();
  void addIMUState(IMUSTATE imu);
  void addVisionState(SE3 T_c_w);
  bool estimateBias(void);
  void getState(SE3& pose, Vec3& vel);
};

#endif // VI_MOTION_H

#ifndef IMU_STATE_H
#define IMU_STATE_H

#include "include/common.h"

class IMUSTATE
{
public:
  Vec3 acc_raw;
  Vec3 gyro_raw;
  double timestamp;

  IMUSTATE();
  IMUSTATE(double secs, double ax, double ay, double az, double gx, double gy, double gz);
  IMUSTATE(double secs, Vec3 acc, Vec3 gyro);
};

#endif // IMU_STATE_H

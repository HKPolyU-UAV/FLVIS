#include "include/imu_state.h"

IMUSTATE::IMUSTATE()
{

}

IMUSTATE::IMUSTATE(double secs, double ax, double ay, double az, double gx, double gy, double gz)
{
  acc_raw = Vec3(ax,ay,az);
  gyro_raw = Vec3(gx,gy,gz);
  timestamp = secs;
}

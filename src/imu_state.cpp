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

IMUSTATE::IMUSTATE(double secs, Vec3 acc, Vec3 gyro)
{
    acc_raw = acc;
    gyro_raw = gyro;
    timestamp = secs;
}

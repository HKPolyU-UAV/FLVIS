#ifndef VI_MOTION_H
#define VI_MOTION_H

#include <include/common.h>
#include <include/imu_state.h>
#include <include/kinetic_math.h>
#include <deque>

#define ACC_MEASUREMENT_FEEDBACK_P (0.1)
#define MAGNITUDE_OF_GRAVITY (9.81)

struct MOTION_STATE{
    Vec3 pos;
    Vec3 vel;
    Quaterniond q_w_i;//rotation of imu in world frame
    IMUSTATE imu_data;
};

class VIMOTION
{
public:
  Vec3 acc_bias;
  Vec3 gyro_bias;
  Vec3 gravity;
  std::deque<MOTION_STATE> states;

  MOTION_STATE init_state;
  bool imu_initialized;

  bool is_first_data;

  void imu_initialization(IMUSTATE imu_read);

  //this is the vimotion trigger, the module will be triggered when the first vision frame come
  //after that the pos vel and orientation will be integrate
  void vision_trigger(Quaterniond& init_orientation);


  VIMOTION();
  void imu_propagation(IMUSTATE imu_read);
  void imu_correction_from_vision(SE3 T_c_w_vision, Vec3 vec_vision);
  void getState(SE3& pose, Vec3& vel);
private:

};

#endif // VI_MOTION_H

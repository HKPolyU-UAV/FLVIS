#ifndef VI_MOTION_H
#define VI_MOTION_H

#include <include/common.h>
#include <include/imu_state.h>
#include <include/kinetic_math.h>
#include <deque>
#include <mutex>

#define ACC_MEASUREMENT_FEEDBACK_P (0.1)
#define MAGNITUDE_OF_GRAVITY       (9.81)
#define STATES_QUEUE_SIZE          (400)

struct MOTION_STATE{
    Vec3 pos;
    Vec3 vel;
    Quaterniond q_w_i;//rotation of imu in world frame
    IMUSTATE imu_data;
};

class VIMOTION
{
public:
    SE3  T_i_c;//transformation from camera frame to imu frame
    SE3  T_c_i;

    Vec3 acc_bias;
    Vec3 gyro_bias;
    Vec3 gravity;
    std::deque<MOTION_STATE> states;
    std::mutex mtx_states_RW;

    MOTION_STATE init_state;
    bool imu_initialized;

    bool is_first_data;

    VIMOTION(SE3 T_i_c_fromCalibration);

    void viIMUinitialization(const IMUSTATE imu_read);
    void viIMUPropagation(const IMUSTATE imu_read);

    //this is the vimotion trigger, the module will be triggered when the first vision frame come
    //after that the pos vel and orientation will be integrate
    void viVisiontrigger(Quaterniond& init_orientation);
    void viVisionRPCompensation(const double time, SE3& T_c_w, double proportion);
    void viGetLatestImuState(SE3& T_w_i, Vec3& vel);

    void viCorrectionFromVision(const double time, SE3 T_c_w_vision, Vec3 vec_vision);
    bool viGetIMURollPitchAtTime(const double time, double& roll, double& pitch);

private:

    bool viFindStateIdx(const double time, int& idx_in_q);


};

#endif // VI_MOTION_H

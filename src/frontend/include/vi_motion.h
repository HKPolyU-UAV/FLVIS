#ifndef VI_MOTION_H
#define VI_MOTION_H

#include <include/common.h>
#include <include/imu_state.h>
#include <include/kinetic_math.h>
#include <deque>
#include <mutex>

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
    double para_1;//Madgwick beta
    double para_2;//proportion of vision feedforware(roll and pich)
    double para_3;//acc-bias feedback parameter
    double para_4;//gyro-bias feedback parameter
    double magnitude_g;//magnitude of gravity
    double ba_sat,bw_sat;
    Vec3 acc_bias;
    Vec3 gyro_bias;
    Vec3 gravity;
    std::deque<MOTION_STATE> states;
    std::mutex mtx_states_RW;

    MOTION_STATE init_state;
    bool imu_initialized;

    bool is_first_data;

    VIMOTION(SE3 T_i_c_fromCalibration,
             double magnitude_g_in = 9.81,
             double para_1_in = 0.1,   //Madgwick beta
             double para_2_in = 0.05,  //proportion of vision feedforware(roll and pich)
             double para_3_in = 0.01,  //acc-bias feedback parameter
             double para_4_in = 0.01,  //gyro-bias feedback parameter
             double para_5_in = 0.5,   //acc bias saturation level
             double para_6_in = 0.1);  //gyro bias saturation level

    void viIMUinitialization(const IMUSTATE imu_read,
                             Quaterniond& q_w_i,
                             Vec3& pos_w_i,
                             Vec3& vel_w_i);
    void viIMUPropagation(const IMUSTATE imu_read,
                          Quaterniond& q_w_i,
                          Vec3& pos_w_i,
                          Vec3& vel_w_i);

    //this is the vimotion trigger, the module will be triggered when the first vision frame come
    //after that the pos vel and orientation will be integrate
    void viVisiontrigger(Quaterniond& init_orientation);
    void viVisionRPCompensation(const double time, SE3& T_c_w);
    void viGetLatestImuState(SE3& T_w_i, Vec3& vel);//latest imu state in queue
    bool viGetCorrFrameState(const double time, SE3& T_c_w);//get correspond frame time

    void viCorrectionFromVision(const double t_curr, const SE3 Tcw_curr,
                                const double t_last, const SE3 Tcw_last,
                                const double err);
    bool viGetIMURollPitchAtTime(const double time, double& roll, double& pitch);

private:

    bool viFindStateIdx(const double time, int& idx_in_q);


};

#endif // VI_MOTION_H

#include "include/vi_motion.h"

VIMOTION::VIMOTION(SE3 T_i_c_fromCalibration)
{
    this->T_i_c = T_i_c_fromCalibration;
    this->T_c_i = this->T_i_c.inverse();

    this->acc_bias=Vec3(0,0,0);
    this->gyro_bias=Vec3(0,0,0);

    this->init_state.pos=Vec3(0,0,0);
    this->init_state.vel=Vec3(0,0,0);
    this->init_state.q_w_i = Vec4(1,0,0,0);

    this->imu_initialized = false;
    this->is_first_data = true;
    this->gravity = Vec3(0,0,-MAGNITUDE_OF_GRAVITY);
}

void VIMOTION::viIMUinitialization(const IMUSTATE imu_read)
{
    //estimate pitch and roll
    this->init_state.imu_data = imu_read;
    Vec3 acc = imu_read.acc_raw;
    Vec3 gyro = imu_read.gyro_raw;
    if(this->is_first_data)
    {
        Vec3 rpy;
        if(fabs(sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2])-MAGNITUDE_OF_GRAVITY)<0.5)//valid acc data
        {//feedback
            cout << "init_imu: ax:" << acc[0] << " ay:" << acc[1] << " az:" << acc[2] << endl;
            rpy[0] = atan2(-acc[1],-acc[2]);
            rpy[1] = atan2(acc[0],-acc[2]);
            rpy[2] = 0;
            this->init_state.q_w_i = rpy2Q(rpy);
            cout << "roll:"   << rpy[0]*57.2958
                 << " pitch:" << rpy[1]*57.2958
                 << " yaw:"   << rpy[2]*57.2958 << endl;
        }
        this->states.push_back(init_state);
        if(states.size()>=STATES_QUEUE_SIZE) {states.pop_front();}
        this->is_first_data =  false;
    }else
    {   //In the initialization process only update orientation, the position and velocity remain 0.
        //madgwick orientation filter
        //Link: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
        //cout << "ax:" << acc[0] << " ay:" << acc[1] << " az:" << acc[2] << endl;
        //cout << "gx:" << gyro[0] << " gy:" << gyro[1] << " gz:" << gyro[2] << endl;
        //gyro update;
        double dt = imu_read.timestamp - this->states.back().imu_data.timestamp;
        //cout << dt << endl;
        Quaterniond q_prev = this->states.back().q_w_i;
        Quaterniond omega(0,gyro[0],gyro[1],gyro[2]);
        Quaterniond qdot = scalar_multi_q(0.5*dt,q1_multi_q2(q_prev,omega));
        Quaterniond q_new = q_plus_q(q_prev,qdot);
        q_new.normalize();
        Vec3 rpy;
        rpy = Q2rpy(q_new);
        //        cout << "roll:"   << rpy[0]*57.2958
        //             << " pitch:" << rpy[1]*57.2958
        //             << " yaw:"   << rpy[2]*57.2958 << endl;

        //acc feedback if acc is valid
        if(fabs(sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2])-MAGNITUDE_OF_GRAVITY)<0.3)//valid acc data
        {//feedback
            Vec3 rpy_acc;
            rpy_acc[0] = atan2(-acc[1],-acc[2]);
            rpy_acc[1] = atan2(acc[0],-acc[2]);
            rpy_acc[2] = 0;
            rpy[0] += (rpy_acc[0]-rpy[0])*0.1;
            rpy[1] += (rpy_acc[1]-rpy[1])*0.1;
            q_new = rpy2Q(rpy);
            //            cout << "roll:"   << rpy[0]*57.2958
            //                 << " pitch:" << rpy[1]*57.2958
            //                 << " yaw:"   << rpy[2]*57.2958 << endl;
        }
        this->init_state.q_w_i = q_new;
        this->states.push_back(init_state);
        if(states.size()>=STATES_QUEUE_SIZE) {states.pop_front();}
        if(states.size()>90)
        {
            this->imu_initialized = true;
        }//end if IMU init
    }
}

void VIMOTION::viVisiontrigger(Quaterniond &init_orientation)
{
    MOTION_STATE state = this->states.back();
    state.pos = Vec3(0,0,0);
    state.vel = Vec3(0,0,0);
    //reset yaw angle to zero
    Vec3 rpy = Q2rpy(state.q_w_i);
    rpy[2] = 0;
    cout << "roll:"   << rpy[0]*57.2958
         << " pitch:" << rpy[1]*57.2958
         << " yaw:"   << rpy[2]*57.2958 << endl;
    Quaterniond q = rpy2Q(rpy);
    cout << q.w() << ","
         << q.x() << ","
         << q.y() << ","
         << q.z() << endl;
    q.normalize();
    cout << q.w() << ","
         << q.x() << ","
         << q.y() << ","
         << q.z() << endl;
    state.q_w_i = q;

    states.clear();
    states.push_back(state);
    init_orientation = state.q_w_i;
    cout << init_orientation.w() << ","
         << init_orientation.x() << ","
         << init_orientation.y() << ","
         << init_orientation.z() << endl;
}

void VIMOTION::viIMUPropagation(const IMUSTATE imu_read)
{
    double dt = imu_read.timestamp - this->states.back().imu_data.timestamp;
    //cout << dt << endl;
    MOTION_STATE s_prev,s_new;//previous state and new state
    Vec3 acc, gyro;
    acc = imu_read.acc_raw-this->acc_bias;
    gyro = imu_read.gyro_raw-this->gyro_bias;
    s_prev = states.back();
    Quaterniond q_prev = s_prev.q_w_i;
    Mat3x3 R_prev = q_prev.toRotationMatrix();
    Vec3 p_prev = s_prev.pos;
    Vec3 v_prev = s_prev.vel;
    Vec3 p_dot,v_dot;

    Quaterniond omega(0,gyro[0],gyro[1],gyro[2]);
    Quaterniond qdot = scalar_multi_q(0.5*dt,q1_multi_q2(q_prev,omega));
    Quaterniond q_new = q_plus_q(q_prev,qdot);
    q_new.normalize();
    if(fabs(sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2])-MAGNITUDE_OF_GRAVITY)<0.3)//valid acc data
    {//feedback
        Vec3 rpy = Q2rpy(q_new);
        Vec3 rpy_acc;
        rpy_acc[0] = atan2(-acc[1],-acc[2]);
        rpy_acc[1] = atan2(acc[0],-acc[2]);
        rpy[0] = (rpy[0]*0.9)+(rpy_acc[0]*0.1);
        rpy[1] = (rpy[1]*0.9)+(rpy_acc[1]*0.1);
        rpy_acc[2] = 0;
        q_new = rpy2Q(rpy);
    }

    v_dot = ((R_prev*acc)-gravity)*dt;
    p_dot = v_prev*dt;

    s_new.q_w_i = q_new;
    s_new.pos = p_prev+p_dot;
    //s_new.vel = v_prev+v_dot;
    s_new.vel = Vec3(0,0,0);
    s_new.imu_data = imu_read;
    //propagation for q
    this->mtx_states_RW.lock();
    states.push_back(s_new);
    if(states.size()>=STATES_QUEUE_SIZE) {states.pop_front();}
    this->mtx_states_RW.unlock();

}
//this->mtx_states_RW.lock();
//this->mtx_states_RW.unlock();
void VIMOTION::viCorrectionFromVision(const double time, SE3 T_c_w_vision, Vec3 vec_vision)
{
    Vec3 acc_bias_est;
    Vec3 gyro_bias_est;
    this->mtx_states_RW.lock();
    int idx;
    this->viFindStateIdx(time, idx);
    SE3 T_w_i_orig=SE3(SO3(states.at(idx).q_w_i),states.at(idx).pos);
    SE3 T_w_i_corr=T_c_w_vision.inverse()*this->T_c_i;
    SE3 T_corr_orig= T_w_i_corr*T_w_i_orig.inverse();
    int qsize=states.size();
    for (;idx<qsize;idx++) {
        SE3 T_orig = SE3(SO3(states.at(idx).q_w_i),states.at(idx).pos);
        SE3 T_corr = T_corr_orig*T_orig;
        states.at(idx).q_w_i = T_corr.unit_quaternion();
        states.at(idx).pos = T_corr.translation();
    };
    //    Vec3 pos = T_w_i_corr.translation();
    //    cout << pos(0) << " " << pos(1) << " " <<pos(2) << endl;
    //    pos = states.back().pos;
    //    cout << pos(0) << " " << pos(1) << " " <<pos(2) << endl;
    this->mtx_states_RW.unlock();
    //correct
    this->acc_bias+=acc_bias_est*0.5;
    this->gyro_bias+=gyro_bias_est*0.5;
    for(size_t i=0; i<3; i++)//bias bound
    {
        if(acc_bias[i]>0.1) acc_bias[i]=0.1;
        if(acc_bias[i]<-0.1) acc_bias[i]=-0.1;
        if(gyro_bias[i]>0.05) gyro_bias[i]=0.05;
        if(gyro_bias[i]<-0.05) gyro_bias[i]=-0.05;
    }
}


// 1   2   3   4   5   6   7   8   9   10   //states queue
//                   |                      //the state
// return the state 5
bool VIMOTION::viFindStateIdx(const double time, int& idx_in_q)
{
    bool ret;
    int idx=9999;

    for(int i=states.size()-1; i>=0; i--)
    {
        if((states.at(i).imu_data.timestamp-time)>0)
        {
            idx = i;
        }
        else
        {
            idx = i;
            break;
        }
    }
    //    cout << "Frame time  :" << std::setprecision (15) << time << endl;
    //    cout << "q begin time:" << this->states.front().imu_data.timestamp << endl;
    //    cout << "q end time  :" << this->states.back().imu_data.timestamp << endl;
    //    cout << "queue   size:" << states.size() << endl;
    //    cout << "idx in queue:" << idx << endl;
    //    cout << "      time  :" << this->states.at(idx).imu_data.timestamp << endl;

    if(idx>0 && idx!=9999)
    {
        idx_in_q = idx;
        ret = true;
    }
    else {
        ret = false;
    }
    return ret;
}


bool VIMOTION::viGetIMURollPitchAtTime(const double time, double &roll, double &pitch)
{
    MOTION_STATE state;
    bool found;

    this->mtx_states_RW.lock();
    int idx;

    if(this->viFindStateIdx(time,idx))
    {
        state=states.at(idx);
        SE3 T_w_i = SE3(state.q_w_i,state.pos);
        Vec3 rpy= Q2rpy(T_w_i.so3().unit_quaternion());
        roll  = rpy[0];
        pitch = rpy[1];
        found = true;
    }else
    {
        cout << "[Critical Warning]: motion not in queue! please enlarge the buffer size" << endl;
        //        cout << "Frame time   :" << std::setprecision (15) << time << endl;
        //        cout << "q begin time :" << this->states.front().imu_data.timestamp << endl;
        //        cout << "q end time   :" << this->states.back().imu_data.timestamp << endl;
        found = false;
    }
    this->mtx_states_RW.unlock();

    return found;
}

void VIMOTION::viGetLatestImuState(SE3 &T_w_i, Vec3 &vel)
{
    this->mtx_states_RW.lock();
    T_w_i = SE3(SO3(states.back().q_w_i),states.back().pos);
    vel   = states.back().vel;
    this->mtx_states_RW.unlock();
}

bool VIMOTION::viVisionRPCompensation(const double time, SE3 &T_c_w, double proportion)
{
    //cout << "In Camera_rp_compensation" << endl;

    Vec3 rpy_before, rpy_vimotion, ryp_after;//ryp_w_c

    SE3 T_c_w_before = T_c_w;
    SE3 T_w_i_before = (T_c_w_before.inverse())*this->T_c_i;
    rpy_before = Q2rpy(T_w_i_before.so3().unit_quaternion());

    if(this->viGetIMURollPitchAtTime(time,rpy_vimotion[0],rpy_vimotion[1]))
    {
        rpy_vimotion[2]=rpy_before[2];
        ryp_after = (rpy_before*(1-proportion))+(rpy_vimotion*proportion);
        //        cout << "b-roll :"  << rpy_before[0]*57.2958 << endl
        //             << "b-pitch:"  << rpy_before[1]*57.2958 << endl
        //             << "b-yaw  :"  << rpy_before[2]*57.2958 << endl;
        //        cout << "i-roll :"  << rpy_vimotion[0]*57.2958 << endl
        //             << "i-pitch:"  << rpy_vimotion[1]*57.2958 << endl
        //             << "i-yaw  :"  << rpy_vimotion[2]*57.2958 << endl;
        //        cout << "a-roll :"  << ryp_after[0]*57.2958 << endl
        //             << "a-pitch:"  << ryp_after[1]*57.2958 << endl
        //             << "a-yaw  :"  << ryp_after[2]*57.2958 << endl;
        SE3 T_w_i_after = SE3(SO3(rpy2Q(ryp_after)),T_w_i_before.translation());
        SE3 T_c_w_after= (T_w_i_after*this->T_i_c).inverse();
        T_c_w = T_c_w_after;
        //cout << "Camera_rp_compensation" << endl;
        return true;
    }else
    {
        cout << "No Cali" << endl;
        return false;
    }

}

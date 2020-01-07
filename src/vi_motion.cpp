#include "include/vi_motion.h"

VIMOTION::VIMOTION()
{
    this->acc_bias=Vec3(0,0,0);
    this->gyro_bias=Vec3(0,0,0);

    this->init_state.pos=Vec3(0,0,0);
    this->init_state.vel=Vec3(0,0,0);
    this->init_state.q_w_i = Vec4(1,0,0,0);

    this->imu_initialized = false;
    this->is_first_data = true;
    this->gravity = Vec3(0,0,-MAGNITUDE_OF_GRAVITY);
}

void VIMOTION::imu_initialization(const IMUSTATE imu_read)
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
        if(states.size()>=100) {states.pop_front();}
        this->is_first_data =  false;
    }else
    {   //In the initialization process only update orientation, the position and velocity remain 0.
        //madgwick orientation filter
        //Link: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
        //cout << "ax:" << acc[0] << " ay:" << acc[1] << " az:" << acc[2] << endl;
        //cout << "gx:" << gyro[0] << " gy:" << gyro[1] << " gz:" << gyro[2] << endl;
        //gyro update;
        double dt = imu_read.timestamp - this->states.back().imu_data.timestamp;
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
        if(states.size()>=100) {states.pop_front();}
        if(states.size()>90)
        {
            this->imu_initialized = true;
        }//end if IMU init
    }
}

void VIMOTION::vision_trigger(Quaterniond &init_orientation)
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

void VIMOTION::imu_propagation(IMUSTATE imu_read)
{
    double dt = imu_read.timestamp - this->states.back().imu_data.timestamp;
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
        rpy_acc[2] = 0;
        rpy[0] += (rpy_acc[0]-rpy[0])*0.1;
        rpy[1] += (rpy_acc[1]-rpy[1])*0.1;
        q_new = rpy2Q(rpy);
    }

    v_dot = ((R_prev*acc)-gravity)*dt;
    p_dot = v_prev*dt;

    s_new.q_w_i = q_new;
    s_new.pos = p_prev+p_dot;
    s_new.vel = v_prev+v_dot;
    s_new.imu_data = imu_read;
    //propagation for q
    states.push_back(s_new);
    if(states.size()>=100) {states.pop_front();}
}

void VIMOTION::imu_correction_from_vision(SE3 T_c_w_vision, Vec3 vec_vision)
{
    Vec3 acc_bias_est;
    Vec3 gyro_bias_est;

    //estimation of gyro bias
    this->acc_bias+=acc_bias_est*0.5;
    this->gyro_bias+=gyro_bias_est*0.5;
    for(size_t i=0; i<3; i++)//bias bound
    {
        if(acc_bias[i]>0.2) acc_bias[i]=0.2;
        if(acc_bias[i]<-0.2) acc_bias[i]=-0.2;
        if(gyro_bias[i]>0.2) gyro_bias[i]=0.2;
        if(gyro_bias[i]<-0.2) gyro_bias[i]=-0.2;
    }

}


#include "include/vi_motion.h"

VIMOTION::VIMOTION(SE3 T_i_c_fromCalibration,
                   double magnitude_g_in,
                   double para_1_in,   //Madgwick beta
                   double para_2_in,  //proportion of vision feedforware(roll and pich)
                   double para_3_in,  //acc-bias feedback parameter
                   double para_4_in,
                   double para_5_in,
                   double para_6_in) //gyro-bias feedback parameter)
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
    this->magnitude_g = magnitude_g_in;
    this->gravity = Vec3(0,0,-magnitude_g);
    this->para_1=para_1_in;
    this->para_2=para_2_in;
    this->para_3=para_3_in;
    this->para_4=para_4_in;
    this->ba_sat=para_5_in;
    this->bw_sat=para_6_in;
}

void VIMOTION::viIMUinitialization(const IMUSTATE imu_read,
                                   Quaterniond& q_w_i,
                                   Vec3& pos_w_i,
                                   Vec3& vel_w_i)
{
    q_w_i = Quaterniond(1,0,0,0);
    pos_w_i = vel_w_i = Vec3(0,0,0);
    init_state.imu_data = imu_read;
    init_state.pos = pos_w_i;
    init_state.vel = vel_w_i;
    Vec3 acc = imu_read.acc_raw-acc_bias;
    Vec3 gyro = imu_read.gyro_raw-gyro_bias;
    if(this->is_first_data)
    {
        Vec3 rpy;
        if((acc.norm()-magnitude_g)<0.3)//valid acc data
        {//feedback
            rpy[0] = atan2(-acc[1],-acc[2]);
            rpy[1] = atan2(acc[0],-acc[2]);
            rpy[2] = 0;
            this->init_state.q_w_i = rpy2Q(rpy);
            cout << "Fist IMU: "
                 << "roll:"   << rpy[0]*57.2958
                 << " pitch:" << rpy[1]*57.2958
                 << " yaw:"   << rpy[2]*57.2958 << endl;
            this->states.push_back(init_state);
            if(states.size()>=STATES_QUEUE_SIZE) {states.pop_front();}
            this->is_first_data =  false;
            q_w_i = rpy2Q(rpy);
        }
    }else
    {   //In the initialization process only update orientation, the position and velocity remain 0.
        //madgwick orientation filter
        //Link: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

        //gyro update;
        double dt = imu_read.timestamp - this->states.back().imu_data.timestamp;
        Quaterniond q_prev = this->states.back().q_w_i;
        Quaterniond omega(0,gyro[0],gyro[1],gyro[2]);
        Quaterniond qdot = scalar_multi_q(0.5,q1_multi_q2(q_prev,omega));
        double acc_norm=acc.norm();
        if((acc_norm-magnitude_g)<0.3)//valid acc data
        {
            // Madgwick cradient decent correction step
            // Normalise accelerometer measurement
            double ax=acc[0]/acc_norm;
            double ay=acc[1]/acc_norm;
            double az=acc[2]/acc_norm;
            double qw=q_prev.w();
            double qx=q_prev.x();
            double qy=q_prev.y();
            double qz=q_prev.z();
            //feed back
            Vec4 s;
            s[0] = 2*qx*(ay + 2*qw*qx + 2*qy*qz) - 2*qy*(ax - 2*qw*qy + 2*qx*qz);
            s[1] = 2*qw*(ay + 2*qw*qx + 2*qy*qz) + 2*qz*(ax - 2*qw*qy + 2*qx*qz) - 4*qx*(- 2*qx*qx - 2*qy*qy + az + 1);
            s[2] = 2*qz*(ay + 2*qw*qx + 2*qy*qz) - 2*qw*(ax - 2*qw*qy + 2*qx*qz) - 4*qy*(- 2*qx*qx - 2*qy*qy + az + 1);
            s[3] = 2*qx*(ax - 2*qw*qy + 2*qx*qz) + 2*qy*(ay + 2*qw*qx + 2*qy*qz);
            s*=s.norm();
            // Apply feedback step
            qdot.w() -= 10*para_1 * s[0];
            qdot.x() -= 10*para_1 * s[1];
            qdot.y() -= 10*para_1 * s[2];
            qdot.z() -= 10*para_1 * s[3];
        }
        Quaterniond q_new = q_plus_q(q_prev,scalar_multi_q(dt,qdot));
        q_new.normalize();
        this->init_state.q_w_i = q_new;
        this->states.push_back(init_state);
        Vec3 rpy =  Q2rpy(states.back().q_w_i);
        if(states.size()>=STATES_QUEUE_SIZE) {states.pop_front();}
        if(states.size()>30)
        {
            Vec3 rpy =  Q2rpy(states.back().q_w_i);
            cout << "Initialized IMU: "
                 << "roll:"   << rpy[0]*57.2958
                 << " pitch:" << rpy[1]*57.2958
                 << " yaw:"   << rpy[2]*57.2958 << endl;
            this->imu_initialized = true;
        }//end if IMU init
    }
}

void VIMOTION::viVisiontrigger(Quaterniond &init_orientation)
{
    this->mtx_states_RW.lock();
    MOTION_STATE state = this->states.back();
    state.pos = Vec3(0,0,0);
    state.vel = Vec3(0,0,0);
    //reset yaw angle to zero
    Vec3 rpy = Q2rpy(state.q_w_i);
    rpy[2] = 0;
    Quaterniond q = rpy2Q(rpy);
    q.normalize();
    state.q_w_i = q;
    states.clear();
    states.push_back(state);
    this->mtx_states_RW.unlock();
    cout << "Vision Trigger at: "
         << "roll:"   << rpy[0]*57.2958
         << " pitch:" << rpy[1]*57.2958
         << " yaw:"   << rpy[2]*57.2958 << endl;
    init_orientation = state.q_w_i;
}

void VIMOTION::viIMUPropagation(const IMUSTATE imu_read,
                                Quaterniond& q_w_i,
                                Vec3& pos_w_i,
                                Vec3& vel_w_i)
{
    MOTION_STATE s_prev,s_new;//previous state and new state
    Vec3 acc, gyro;
    acc =  imu_read.acc_raw  - acc_bias;
    gyro = imu_read.gyro_raw - gyro_bias;


    this->mtx_states_RW.lock();

    s_prev = states.back();
    double dt = imu_read.timestamp - s_prev.imu_data.timestamp;
    Quaterniond q_prev = s_prev.q_w_i;
    Mat3x3      R_prev = q_prev.toRotationMatrix();
    Vec3        p_prev = s_prev.pos;
    Vec3        v_prev = s_prev.vel;
    Vec3        p_dot,v_dot;

    //propagation for q
    Quaterniond omega(0,gyro[0],gyro[1],gyro[2]);
    Quaterniond qdot = scalar_multi_q(0.5,q1_multi_q2(q_prev,omega));
    double acc_norm=acc.norm();
    if((acc_norm-magnitude_g)<0.3)//valid acc data
    {
        // Madgwick cradient decent correction step
        // Normalise accelerometer measurement
        double ax=acc[0]/acc_norm;
        double ay=acc[1]/acc_norm;
        double az=acc[2]/acc_norm;
        double qw=q_prev.w();
        double qx=q_prev.x();
        double qy=q_prev.y();
        double qz=q_prev.z();
        //feed back
        Vec4 s;
        s[0] = 2*qx*(ay + 2*qw*qx + 2*qy*qz) - 2*qy*(ax - 2*qw*qy + 2*qx*qz);
        s[1] = 2*qw*(ay + 2*qw*qx + 2*qy*qz) + 2*qz*(ax - 2*qw*qy + 2*qx*qz) - 4*qx*(- 2*qx*qx - 2*qy*qy + az + 1);
        s[2] = 2*qz*(ay + 2*qw*qx + 2*qy*qz) - 2*qw*(ax - 2*qw*qy + 2*qx*qz) - 4*qy*(- 2*qx*qx - 2*qy*qy + az + 1);
        s[3] = 2*qx*(ax - 2*qw*qy + 2*qx*qz) + 2*qy*(ay + 2*qw*qx + 2*qy*qz);
        s*=s.norm();
        // Apply feedback step
        qdot.w() -= para_1 * s[0];
        qdot.x() -= para_1 * s[1];
        qdot.y() -= para_1 * s[2];
        qdot.z() -= para_1 * s[3];
    }
    Quaterniond q_new = q_plus_q(q_prev,scalar_multi_q(dt,qdot));
    q_new.normalize();
    s_new.q_w_i = q_new;

    //propagation for pos
    p_dot = v_prev*dt;
    s_new.pos = p_prev+p_dot;

    //propagation for vel
    v_dot = ((R_prev*acc)-gravity)*dt;

    s_new.vel = v_prev+v_dot;
    s_new.imu_data = imu_read;


    states.push_back(s_new);
    if(states.size()>=STATES_QUEUE_SIZE) {states.pop_front();}
    this->mtx_states_RW.unlock();
    q_w_i   = s_new.q_w_i;
    pos_w_i = s_new.pos;
    vel_w_i = s_new.vel;
}
//this->mtx_states_RW.lock();
//this->mtx_states_RW.unlock();
void VIMOTION::viCorrectionFromVision(const double t_curr, const SE3 Tcw_curr,
                                      const double t_last, const SE3 Tcw_last,
                                      const double err)
{
    Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "[", "]");
    Vec3 acc_bias_est=Vec3(0,0,0);
    Vec3 gyro_bias_est=Vec3(0,0,0);

    this->mtx_states_RW.lock();
    //TIME A--------------------B(VISION)            B--------------------C
    //     a ------- m -------  b(IMU)            b->B ------- m -------  c
    //     A and a are aligned
    double dt = t_curr-t_last;
    int  idx_curr,idx_last,idx_mid;
    if(     //Linearization in short period of time
            viFindStateIdx(t_last, idx_last)
            &&viFindStateIdx(t_curr, idx_curr))
    {
        double dt = t_curr-t_last;
        idx_mid = idx_last+floor((idx_curr-idx_last)/2);

        //vision
        SE3          T_w_iA =  (Tcw_last.inverse())*T_c_i;
        SE3          T_w_iB =  (Tcw_curr.inverse())*T_c_i;
        //IMU
        SE3          T_w_ia = SE3(states.at(idx_last).q_w_i,states.at(idx_last).pos);
        SE3          T_w_ib = SE3(states.at(idx_curr).q_w_i,states.at(idx_curr).pos);
        SE3          T_w_im = SE3(states.at(idx_mid).q_w_i,states.at(idx_mid).pos);
        //Diff
        SE3          T_iB_iA = (T_w_iB.inverse()) * T_w_iA;
        SE3          T_ib_ia = (T_w_ib.inverse()) * T_w_ia;

        //gyro bias
        Quaterniond  Q_B_A = T_iB_iA.unit_quaternion();
        Quaterniond  Q_b_a = T_ib_ia.unit_quaternion();
        Quaterniond  Q_B_b = Q_B_A*(Q_b_a.inverse());
        gyro_bias_est[0] = Q_B_b.x()/dt;
        gyro_bias_est[1] = Q_B_b.y()/dt;
        gyro_bias_est[2] = Q_B_b.z()/dt;

        //acc bias
        //        Vec3         p_b_B = T_w_iB.translation()-T_w_ib.translation();
        //        Vec3         p_b_B_local = (T_w_im.unit_quaternion().inverse().toRotationMatrix())*p_b_B;////in imu frame
        //        acc_bias_est = -p_b_B_local/(0.5*dt*dt);

        //acc bias(inter-frame)
        int cnt = idx_curr-idx_last+1;
        Vec3         vel_imu(0,0,0);
        for(int i=idx_last; i<=idx_curr ; i++)
        {
            vel_imu+=states.at(i).vel;
        }
        vel_imu *= (1.0/cnt);
        Vec3         vel_vision_world = (T_w_iB.translation()-T_w_iA.translation())/dt;
        Vec3         diff_vel_world=vel_vision_world-vel_imu;
        Vec3         diff_vel_local = (T_w_im.unit_quaternion().inverse().toRotationMatrix())*diff_vel_world;
        acc_bias_est = -(diff_vel_local)/dt;

        SE3 T_diff = T_w_iB*(T_w_ib.inverse());
        for(int i=idx_curr; i<states.size(); i++)
        {
            SE3 newT = T_diff*SE3(states.at(i).q_w_i,states.at(i).pos);
            states.at(i).q_w_i = newT.unit_quaternion();
            states.at(i).pos   = newT.translation();
            states.at(i).vel += diff_vel_world;
        }
        //        cout << "V_pre_R: " << (57*Q2rpy(qwi_last_v)).transpose().format(CleanFmt) << endl;
        //        cout << "V_cur_R: " << (57*Q2rpy(qwi_curr_v)).transpose().format(CleanFmt) << endl;
        //        cout << "I_cur_R: " << (57*Q2rpy(qwi_curr_i)).transpose().format(CleanFmt) << endl;
        //        cout << "DR(V-I): " << (57*(Q2rpy(qwi_curr_v)-Q2rpy(qwi_curr_i))).transpose().format(CleanFmt) << endl;
        //        cout << "DR(V-I): " << (57*Q2rpy(dq_vision_imu)).transpose().format(CleanFmt) << endl;
        //        cout << "V_mid_v: " << vwi_mid_v.transpose().format(CleanFmt) << endl;
        //        cout << "I_mid_v: " << vwi_mid_i.transpose().format(CleanFmt) << endl;
        //        cout << "Dv(V-I)_w: " << dv_vision_imu_wf.transpose().format(CleanFmt) << "with norm"  << dv_vision_imu_wf.norm() <<endl;
        //        cout << "Dv(V-I)_i: " << dv_vision_imu_if.transpose().format(CleanFmt) << "with norm"  << dv_vision_imu_if.norm() <<endl;
        //        cout << "V_mid_R: " << (57*Q2rpy(qwi_mid_v)).transpose().format(CleanFmt) << endl;
        //        cout << "I_mid_R: " << (57*Q2rpy(qwi_mid_i)).transpose().format(CleanFmt) << endl;
        //        cout << "Ba_est : " << acc_bias_est.transpose().format(CleanFmt) << endl;
        //        cout << "Bg_est : " << gyro_bias_est.transpose().format(CleanFmt) << endl;

        double ba_est_norm = acc_bias_est.norm();
        if(ba_est_norm>ba_sat)
        {
            acc_bias_est*=(ba_sat/ba_est_norm);
        }
        double bw_est_norm = gyro_bias_est.norm();
        if(ba_est_norm>bw_sat)
        {
            gyro_bias_est*=(bw_sat/bw_est_norm);
        }

//        for (int i=0; i<3; i++)
//        {
//            if(acc_bias_est[i]>1.0) acc_bias_est[i] = 1.0;
//            if(acc_bias_est[i]<-1.0) acc_bias_est[i] = -1.0;
//            if(gyro_bias_est[i]>0.1) gyro_bias_est[i] = 0.1;
//            if(gyro_bias_est[i]<-0.1) gyro_bias_est[i] = -0.1;
//        }

        if(dt<0.1)
        {
            acc_bias  =  (1-para_3)*acc_bias+(para_3)*acc_bias_est;
            gyro_bias =  (1-para_3)*gyro_bias+(para_4)*gyro_bias_est;
        }
        //correct
//        cout << "acc_bias : " << acc_bias.transpose().format(CleanFmt) << endl;
//        cout << "gyro_bias: " << gyro_bias.transpose().format(CleanFmt) << endl;
    }
    else
    {
        cout << "No Correction" << endl;
    }

    this->mtx_states_RW.unlock();


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

    if(idx>0 && idx!=9999)
    {
        idx_in_q = idx;
        //    cout << "idx in queue:" << idx << endl;
        //    cout << "      time  :" << this->states.at(idx).imu_data.timestamp << endl;
        ret = true;
    }
    else
    {
        cout << "[Critical Warning]: motion not in queue! please enlarge the buffer size" << endl;
        cout << " queue size:" << states.size()
             << " querry time:" << std::setprecision (15) << time
             << " q begin:" << this->states.front().imu_data.timestamp
             << " q end  :" << this->states.back().imu_data.timestamp << endl;
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

bool VIMOTION::viGetCorrFrameState(const double time, SE3 &T_c_w)
{
    bool ret;
    int  idx;
    MOTION_STATE state;
    this->mtx_states_RW.lock();
    if(this->viFindStateIdx(time,idx))
    {
        state=states.at(idx);
        SE3 T_w_i = SE3(state.q_w_i,state.pos);
        SE3 T_w_c = T_w_i * this->T_i_c;
        T_c_w = T_w_c.inverse();
        ret = true;
    }else
    {
        ret = false;
    }
    this->mtx_states_RW.unlock();
    return ret;
}

void VIMOTION::viVisionRPCompensation(const double time, SE3 &T_c_w)
{
    Vec3 rpy_before, rpy_vimotion, ryp_after;//ryp_w_c
    SE3 T_c_w_before = T_c_w;
    SE3 T_w_i_before = (T_c_w_before.inverse())*this->T_c_i;
    rpy_before = Q2rpy(T_w_i_before.so3().unit_quaternion());
    if(this->viGetIMURollPitchAtTime(time,rpy_vimotion[0],rpy_vimotion[1]))
    {
        rpy_vimotion[2]=rpy_before[2];
        ryp_after = (rpy_before*(1-para_2))+(rpy_vimotion*para_2);
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
    }else
    {
        cout << "No RPCompensation" << endl;
    }
    return;
}

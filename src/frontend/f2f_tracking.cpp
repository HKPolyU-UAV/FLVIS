#include "include/f2f_tracking.h"
#include <chrono>


void F2FTracking::init(const int w, const int h,
                       const Mat c0_cameraMatrix_in, const Mat c0_distCoeffs_in,
                       const SE3 T_i_c0_in,
                       const Vec4 vi_para,
                       const TYPEOFCAMERA cam_type_in,
                       const double cam_scale_in,
                       const Mat c1_cameraMatrix_in, const Mat c1_distCoeffs_in,
                       const SE3 T_c0_c1)
{
    this->feature_dem   = new FeatureDEM(w,h,5);
    this->lkorb_tracker = new LKORBTracking(w,h);
    this->vimotion      = new VIMOTION(T_i_c0_in,  9.81,
                                       vi_para[0], vi_para[1],  vi_para[2], vi_para[3]);
    curr_frame = std::make_shared<CameraFrame>();
    last_frame = std::make_shared<CameraFrame>();
    curr_frame->height = last_frame->height = h;
    curr_frame->width = last_frame->width = w;
    this->cam_type = cam_type_in;

    if(cam_type==DEPTH_D435I)
    {
        K0_rect = c0_cameraMatrix_in;
        D0_rect = c0_distCoeffs_in;
        DepthCamera dc;
        dc.setDepthCamInfo(K0_rect.at<double>(0,0),//fx
                           K0_rect.at<double>(1,1),//fy
                           K0_rect.at<double>(0,2),//cx
                           K0_rect.at<double>(1,2),//cy
                           1000.0);
        curr_frame->d_camera = last_frame->d_camera = dc;
    }
    if(cam_type==STEREO_EuRoC_MAV)
    {
        K0 = c0_cameraMatrix_in;
        D0 = c0_distCoeffs_in;
        K1 = c1_cameraMatrix_in;
        D1 = c1_distCoeffs_in;
        SE3 T_c1_c0 = T_c0_c1.inverse();
        Mat3x3 R_ = T_c1_c0.rotation_matrix();
        Vec3   T_ = T_c1_c0.translation();
        cv::Mat R__ = (cv::Mat1d(3, 3) << R_(0,0), R_(0,1), R_(0,2),
                       R_(1,0), R_(1,1), R_(1,2),
                       R_(2,0), R_(2,1), R_(2,2));
        cv::Mat T__ = (cv::Mat1d(3, 1) << T_(0), T_(1), T_(2));
        cv::Mat R0,R1,P0,P1,Q;
        cv::stereoRectify(K0,D0,K1,D1,cv::Size(w,h),R__,T__,
                          R0,R1,P0,P1,Q,
                          CALIB_ZERO_DISPARITY,0,cv::Size(w,h));

        D1_rect = D0_rect = (cv::Mat1d(4, 1) << 0,0,0,0);
        cv::initUndistortRectifyMap(K0,D0,R0,P0,cv::Size(w,h),CV_32F,
                                    c0_RM[0],c0_RM[1]);
        cv::initUndistortRectifyMap(K1,D1,R1,P1,cv::Size(w,h),CV_32F,
                                    c1_RM[0],c1_RM[1]);
        K0_rect = P0.rowRange(0,3).colRange(0,3);
        K1_rect = P1.rowRange(0,3).colRange(0,3);

        DepthCamera dc;
        Mat3x4 P0_,P1_;
        P0_(0,0) = P0.at<double>(0,0);
        P0_(1,1) = P0.at<double>(1,1);
        P0_(0,2) = P0.at<double>(0,2);
        P0_(1,2) = P0.at<double>(1,2);
        P0_(2,2) = P0.at<double>(2,2);
        P0_(0,3) = P0.at<double>(0,3);
        P0_(1,3) = P0.at<double>(1,3);
        P0_(2,3) = P0.at<double>(2,3);

        P1_(0,0) = P1.at<double>(0,0);
        P1_(1,1) = P1.at<double>(1,1);
        P1_(0,2) = P1.at<double>(0,2);
        P1_(1,2) = P1.at<double>(1,2);
        P1_(2,2) = P1.at<double>(2,2);
        P1_(0,3) = P1.at<double>(0,3);
        P1_(1,3) = P1.at<double>(1,3);
        P1_(2,3) = P1.at<double>(2,3);

        dc.setSteroCamInfo(K0_rect, D0_rect, P0_,
                           K1_rect, D1_rect, P1_,
                           T_c0_c1);
        curr_frame->d_camera = last_frame->d_camera = dc;
    }
    this->frameCount = 0;
    this->vo_tracking_state = UnInit;
    this->has_localmap_feedback = false;
}


void F2FTracking::imu_feed(const double time, const Vec3 acc, const Vec3 gyro,
                           Quaterniond &q_w_i, Vec3 &pos_w_i, Vec3 &vel_w_i)
{
    if(!(vimotion->imu_initialized))
    {
        //estimate orientation
        this->has_imu=true;
        vimotion->viIMUinitialization(IMUSTATE(time,acc,gyro),q_w_i,pos_w_i,vel_w_i);
    }else {
        //estimate pos vel and orientation
        vimotion->viIMUPropagation(IMUSTATE(time,acc,gyro),q_w_i,pos_w_i,vel_w_i);
        //vimotion->viIMUPropagation(IMUSTATE(time,Vec3(0,0,-9.8),Vec3(0,0,0.1)),q_w_i,pos_w_i,vel_w_i);
    }
}

void F2FTracking::correction_feed(const double time, const CorrectionInfStruct corr)
{
    this->correction_inf = corr;
    this->has_localmap_feedback = true;
}




bool F2FTracking::init_frame()
{
    bool init_succeed=false;
    if(cam_type==DEPTH_D435I)
    {
        vector<Vec2> pts2d;
        vector<cv::Mat>  descriptors;
        this->feature_dem->detect(curr_frame->img0,pts2d,descriptors);
        cout << "Detect " << pts2d.size() << " Features for init process"<< endl;
        for(size_t i=0; i<pts2d.size(); i++)
        {
            curr_frame->landmarks.push_back(LandMarkInFrame(descriptors.at(i),
                                                            pts2d.at(i),
                                                            Vec3(0,0,0),
                                                            false,
                                                            curr_frame->T_c_w));
        }
        curr_frame->depthInnovation();
        if(curr_frame->validLMCount()>30)
        {
            ID_POSE tmp;
            tmp.frame_id = curr_frame->frame_id;
            tmp.T_c_w = curr_frame->T_c_w;
            pose_records.push_back(tmp);
            T_c_w_last_keyframe = curr_frame->T_c_w;
            init_succeed = true;
            cout << "vo_tracking_state = Tracking" << endl;
        }
    }
    if(cam_type==STEREO_EuRoC_MAV){

        vector<Vec2> pts2d_img0,pts2d_img1;
        vector<cv::Mat>  descriptors;
        this->feature_dem->detect(curr_frame->img0,pts2d_img0,descriptors);
        cout << "Detect " << pts2d_img0.size() << " Features for init process"<< endl;
        for(size_t i=0; i<pts2d_img0.size(); i++)
        {
            curr_frame->landmarks.push_back(LandMarkInFrame(descriptors.at(i),
                                                            pts2d_img0.at(i),
                                                            Vec3(0,0,0),
                                                            false,
                                                            curr_frame->T_c_w));
        }
        curr_frame->depthInnovation();
        if(curr_frame->validLMCount()>30)
        {
            ID_POSE tmp;
            tmp.frame_id = curr_frame->frame_id;
            tmp.T_c_w = curr_frame->T_c_w;
            pose_records.push_back(tmp);
            T_c_w_last_keyframe = curr_frame->T_c_w;
            init_succeed = true;
            cout << "vo_tracking_state = Tracking" << endl;
        }
    }
    return init_succeed;
}

void F2FTracking::image_feed(const double time,
                             const cv::Mat img0_in,
                             const cv::Mat img1_in,
                             bool &new_keyframe,
                             bool &reset_cmd)
{
    auto start = high_resolution_clock::now();
    new_keyframe = false;
    reset_cmd = false;
    frameCount++;

    //    if(frameCount==40)
    //    {
    //        vimotion->imu_initialized = false;
    //        vimotion->is_first_data = true;
    //        vo_tracking_state = UnInit;
    //        reset_cmd = true;
    //        return;
    //    }

    last_frame.swap(curr_frame);
    curr_frame->clear();
    curr_frame->frame_id = frameCount;
    curr_frame->frame_time = time;

    switch(this->cam_type)
    {
    case DEPTH_D435I:
        curr_frame->img0=img0_in;
        curr_frame->d_img=img1_in;
        //cv::equalizeHist(curr_frame->img0,curr_frame->img0);
        break;
    case STEREO_EuRoC_MAV:
        cv::remap(img0_in, curr_frame->img0, c0_RM[0], c0_RM[1],cv::INTER_LINEAR);
        cv::remap(img1_in, curr_frame->img1, c1_RM[0], c1_RM[1],cv::INTER_LINEAR);
        //cv::equalizeHist(curr_frame->img0,curr_frame->img0);
        //cv::equalizeHist(curr_frame->img1,curr_frame->img1);
        break;
    }

    switch(vo_tracking_state)
    {
    case UnInit:
    {
        Mat3x3 R_w_c;
        // 0  0  1
        //-1  0  0
        // 0 -1  0
        R_w_c << 0, 0, 1, -1, 0, 0, 0,-1, 0;
        Vec3   t_w_c=Vec3(0,0,0);
        SE3    T_w_c(R_w_c,t_w_c);
        curr_frame->T_c_w=T_w_c.inverse();//Tcw = (Twc)^-1
        if(this->has_imu)
        {
            if(vimotion->imu_initialized)
            {
                Eigen::Quaterniond q_init_w_i;
                vimotion->viVisiontrigger(q_init_w_i);
                Vec3 rpy = Q2rpy(q_init_w_i);
                cout << "use imu pose for init pose" << endl;
                cout << "roll:"   << rpy[0]*57 << " pitch:" << rpy[1]*57 << " yaw:"   << rpy[2]*57 << endl;
                Mat3x3 R_w_c = q_init_w_i.toRotationMatrix()*vimotion->T_i_c.rotation_matrix();
                SE3    T_w_c(R_w_c,t_w_c);
                rpy = Q2rpy(T_w_c.unit_quaternion());
                cout << "camera in world: " << endl;
                cout << "roll:"   << rpy[0]*57 << " pitch:" << rpy[1]*57 << " yaw:"   << rpy[2]*57 << endl;
                curr_frame->T_c_w=T_w_c.inverse();//Tcw = (Twc)^-1

                SE3 Twi = curr_frame->T_c_w.inverse()*vimotion->T_c_i;
                rpy = Q2rpy(Twi.unit_quaternion());
                cout << "imu in world: " << endl;
                cout << "roll:"   << rpy[0]*57 << " pitch:" << rpy[1]*57 << " yaw:"   << rpy[2]*57 << endl;
            }else
            {
                break;
            }

        }
        if(this->init_frame())
        {
            new_keyframe = true;
            vo_tracking_state = Tracking;
        }
        break;
    }
    case Tracking:
    {
        /* F2F Workflow
                             STEP1: Recover from LocalMap Feedback msg
                             STEP2: Track Match and Update to curr_frame
                             STEP3: 2D3D-PNP
                             (Option) ->IMU roll pitch compensation
                             STEP4: F2FBA
                             (Option) ->IMU state update from vision
                             STEP5: Redetect
                             STEP6: Update Landmarks(IIR)
                             STEP7: Record Pose
                             STEP8: Switch KeyFrame if needed
                    */
        //STEP1:
        if(has_localmap_feedback)
        {
            //find pose;
            int corr_id = correction_inf.frame_id;
            int old_pose_idx = 0;
            for(int i=(pose_records.size()-1); i>=0; i--)
            {
                if(pose_records.at(i).frame_id==corr_id)
                {
                    old_pose_idx=i;
                    break;
                }
            }
            SE3 old_T_c_w = pose_records.at(old_pose_idx).T_c_w;
            SE3 old_T_c_w_inv = old_T_c_w.inverse();
            SE3 update_T_c_w = correction_inf.T_c_w;
            //update pose records
            for(int i=old_pose_idx; i<pose_records.size(); i++)
            {
                SE3 T_diff= pose_records.at(i).T_c_w * old_T_c_w_inv;
                pose_records.at(i).T_c_w = T_diff * update_T_c_w;
            }
            SE3 T_diff= last_frame->T_c_w * old_T_c_w_inv;
            last_frame->T_c_w = T_diff * update_T_c_w;
            last_frame->correctLMP3DWByLMP3DCandT();
            //update last_frame landmake lm_3d_w and mask outlier
            last_frame->forceCorrectLM3DW(correction_inf.lm_count,correction_inf.lm_id,correction_inf.lm_3d);
            last_frame->forceMarkOutlier(correction_inf.lm_outlier_count,correction_inf.lm_outlier_id);
            has_localmap_feedback = false;
        }
        //STEP2:
        vector<Vec2> lm2d_from,lm2d_to,outlier_tracking;
        this->lkorb_tracker->tracking(*last_frame,
                                      *curr_frame,
                                      lm2d_from,
                                      lm2d_to,
                                      outlier_tracking);
        //STEP3:
        vector<cv::Point2f> p2d;
        vector<cv::Point3f> p3d;
        curr_frame->getValid2d3dPair_cvPf(p2d,p3d);
        static int continus_tracking_fail_cnt = 0;//anti flashlight/autoexposure vibration
        if(p2d.size()<=10)
        {
            continus_tracking_fail_cnt++;
            cout << "[Critical Warning] Tracking Fail-no enough lm pairs" << endl;
            last_frame.swap(curr_frame);//dummy swap, escape this frame
            if(continus_tracking_fail_cnt>3)
            {
                vo_tracking_state = TrackingFail;
                cout << "Tracking failed! Swith to tracking Fail Mode" << endl;
                continus_tracking_fail_cnt = 0;
            }
            break;
        }
        continus_tracking_fail_cnt = 0;
        cv::Mat r_ = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat t_ = cv::Mat::zeros(3, 1, CV_64FC1);
        SE3_to_rvec_tvec(last_frame->T_c_w, r_ , t_ );
        cv::Mat inliers;
        solvePnPRansac(p3d,p2d,K0_rect,D0_rect,
                       r_,t_,false,100,3.0,0.99,inliers,cv::SOLVEPNP_ITERATIVE);
        curr_frame->T_c_w = SE3_from_rvec_tvec(r_,t_);
        std::vector<uchar> status;
        for (int i = 0; i < (int)p2d.size(); i++)
            status.push_back(0);
        for( int i = 0; i < inliers.rows; i++)
        {
            int n = inliers.at<int>(i);
            status[n] = 1;
        }
        curr_frame->updateLMState(status);
        //(Option) ->IMU roll pitch compensation
        if(this->has_imu)
        {
            vimotion->viVisionRPCompensation(curr_frame->frame_time, curr_frame->T_c_w);
        }
        //STEP4:
        OptimizeInFrame::optimize(*curr_frame);
        vector<Vec2> outlier_reproject;
        double mean_reprojection_error;
        curr_frame->calReprjInlierOutlier(mean_reprojection_error,outlier_reproject,1.5);
        curr_frame->reprojection_error=mean_reprojection_error;
        curr_frame->eraseReprjOutlier();

        //(Option) ->Vision Feedback and bias estimation
        if(this->has_imu)
        {
            vimotion->viCorrectionFromVision(curr_frame->frame_time,
                                             curr_frame->T_c_w,
                                             last_frame->frame_time,
                                             curr_frame->T_c_w);
        }
        //STEP5:
        vector<Vec2> newKeyPts;
        vector<cv::Mat>  newDescriptor;
        int newPtsCount;
        this->feature_dem->redetect(curr_frame->img0,
                                    curr_frame->get2dPtsVec(),
                                    newKeyPts,newDescriptor,newPtsCount);
        for(size_t i=0; i<newKeyPts.size(); i++)
        {
            curr_frame->landmarks.push_back(LandMarkInFrame(newDescriptor.at(i),
                                                            newKeyPts.at(i),
                                                            Vec3(0,0,0),
                                                            false,
                                                            curr_frame->T_c_w));
        }
        //STEP6:
        curr_frame->depthInnovation();
        //STEP7:
        ID_POSE tmp;
        tmp.frame_id = curr_frame->frame_id;
        tmp.T_c_w = curr_frame->T_c_w;
        pose_records.push_back(tmp);
        if(pose_records.size() >= 1000)
        {
            pose_records.pop_front();
        }
        //STEP8:
        SE3 T_diff_key_curr = T_c_w_last_keyframe*(curr_frame->T_c_w.inverse());
        Vec3 t=T_diff_key_curr.translation();
        Vec3 r=T_diff_key_curr.so3().log();
        double t_norm = fabs(t[0]) + fabs(t[1]) + fabs(t[2]);
        double r_norm = fabs(r[0]) + fabs(r[1]) + fabs(r[2]);
        if(t_norm>=0.03 || r_norm>=0.2)
        {
            new_keyframe = true;
            T_c_w_last_keyframe = curr_frame->T_c_w;
        }
        break;
    }//end of state: Tracking
    case TrackingFail:
    {
        static int cnt=0;
        cnt++;
        if(cnt==15)
        {
            cout << "vision tracking fail, IMU motion only" << endl << "Tring to recover~" << endl;
            vector<Vec2> pts2d;
            vector<cv::Mat>  descriptors;
            this->feature_dem->detect(curr_frame->img0,pts2d,descriptors);
            cout << "Detect " << pts2d.size() << " Features"<< endl;
            if(this->vimotion->viGetCorrFrameState(curr_frame->frame_time,curr_frame->T_c_w))
            {
                for(size_t i=0; i<pts2d.size(); i++)
                {
                    curr_frame->landmarks.push_back(LandMarkInFrame(descriptors.at(i),
                                                                    pts2d.at(i),
                                                                    Vec3(0,0,0),
                                                                    false,
                                                                    curr_frame->T_c_w));
                }
                curr_frame->depthInnovation();
                if(curr_frame->validLMCount()>30)
                {
                    ID_POSE tmp;
                    tmp.frame_id = curr_frame->frame_id;
                    tmp.T_c_w = curr_frame->T_c_w;
                    pose_records.push_back(tmp);
                    T_c_w_last_keyframe = curr_frame->T_c_w;
                    new_keyframe = true;
                    vo_tracking_state = Tracking;
                    cout << "vo_tracking_state = Working" << endl;
                }else
                {
                    cout << "Re-initialization fail: no enough measurement" << endl;
                }
            }
            else
            {
                cout << "Re-initialization fail: can not find the frame in motion module" << endl;
            }
            cnt=0;
        }else
        {
            if((cnt%5)==0)
                reset_cmd = true;
        }

        break;
    }//end of state: TrackingFail
    }//end of state machine

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    curr_frame->solving_time = (duration.count()/1000.0);
}

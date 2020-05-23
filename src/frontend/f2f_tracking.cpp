#include "include/f2f_tracking.h"
#include <chrono>


void F2FTracking::init(const int w, const int h,
                       const Mat c0_cameraMatrix_in, const Mat c0_distCoeffs_in,
                       const SE3 T_i_c0_in,
                       const Vec6 feature_para,
                       const Vec4 vi_para,
                       const TYPEOFCAMERA cam_type_in,
                       const double cam_scale_in,
                       const Mat c1_cameraMatrix_in, const Mat c1_distCoeffs_in,
                       const SE3 T_c0_c1)
{
    this->feature_dem   = new FeatureDEM(w,h,feature_para);
    this->lkorb_tracker = new LKORBTracking(w,h);
    this->vimotion      = new VIMOTION(T_i_c0_in,  9.81,
                                       vi_para[0], vi_para[1],  vi_para[2], vi_para[3]);
    curr_frame = std::make_shared<CameraFrame>();
    last_frame = std::make_shared<CameraFrame>();
    curr_frame->height = last_frame->height = h;
    curr_frame->width = last_frame->width = w;
    this->cam_type = cam_type_in;
    switch (this->cam_type)
    {
    case DEPTH_D435:
    {
        K0_rect = c0_cameraMatrix_in;
        D0_rect = c0_distCoeffs_in;
        DepthCamera dc;
        dc.setDepthCamInfo(K0_rect.at<double>(0,0),//fx
                           K0_rect.at<double>(1,1),//fy
                           K0_rect.at<double>(0,2),//cx
                           K0_rect.at<double>(1,2),//cy
                           1000.0);
        d_camera = lkorb_tracker->d_camera = curr_frame->d_camera = last_frame->d_camera = dc;
        break;
    }
    case STEREO_EuRoC_MAV:{
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
        dc.setSteroCamInfo(K0, D0, K0_rect, D0_rect, R0, P0,
                           K1, D1, K1_rect, D1_rect, R1, P1,
                           T_c0_c1);
        d_camera = lkorb_tracker->d_camera = curr_frame->d_camera = last_frame->d_camera = dc;
        break;
    }
    }
    this->frameCount = 0;
    this->vo_tracking_state = UnInit;
    this->has_localmap_feedback = false;
}

void F2FTracking::correction_feed(const double time, const CorrectionInfStruct corr)
{
    this->correction_inf = corr;
    this->has_localmap_feedback = true;
}

void F2FTracking::imu_feed(const double time, const Vec3 acc, const Vec3 gyro,
                           Quaterniond &q_w_i, Vec3 &pos_w_i, Vec3 &vel_w_i)
{
    if(!(vimotion->imu_initialized))
    {
        this->has_imu=true;
        vimotion->viIMUinitialization(IMUSTATE(time,acc,gyro),q_w_i,pos_w_i,vel_w_i);
    }else
    {
        vimotion->viIMUPropagation(IMUSTATE(time,acc,gyro),q_w_i,pos_w_i,vel_w_i);
    }
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

    last_frame.swap(curr_frame);
    curr_frame->clear();
    curr_frame->frame_id = frameCount;
    curr_frame->frame_time = time;
    bool mbRGB = 0;
    if(img0_in.channels()==3)
    {
        if(mbRGB)
            cvtColor(img0_in,img0_in,CV_RGB2GRAY);
        else
            cvtColor(img0_in,img0_in,CV_BGR2GRAY);
    }
    else if(img0_in.channels()==4)
    {
        if(mbRGB)
            cvtColor(img0_in,img0_in,CV_RGBA2GRAY);
        else
            cvtColor(img0_in,img0_in,CV_BGRA2GRAY);
    }
    if(img1_in.channels()==3)
    {
        if(mbRGB)
            cvtColor(img1_in,img1_in,CV_RGB2GRAY);
        else
            cvtColor(img1_in,img1_in,CV_BGR2GRAY);
    }
    else if(img1_in.channels()==4)
    {
        if(mbRGB)
            cvtColor(img1_in,img1_in,CV_RGBA2GRAY);
        else
            cvtColor(img1_in,img1_in,CV_BGRA2GRAY);
    }


    switch(this->cam_type)
    {
    case DEPTH_D435:
    {
        curr_frame->img0=img0_in;
        curr_frame->d_img=img1_in;
        if(frameCount<60) return;
        //cv::equalizeHist(curr_frame->img0,curr_frame->img0);
        break;
    }

    case STEREO_EuRoC_MAV:
    {
        curr_frame->img0=img0_in;
        curr_frame->img1=img1_in;
        //        cv::remap(img0_in, curr_frame->img0, c0_RM[0], c0_RM[1],cv::INTER_LINEAR);
        //        cv::remap(img1_in, curr_frame->img1, c1_RM[0], c1_RM[1],cv::INTER_LINEAR);
//        cv::equalizeHist(curr_frame->img0,curr_frame->img0);
//        cv::equalizeHist(curr_frame->img1,curr_frame->img1);
        break;
    }
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
                Mat3x3 R_w_c = q_init_w_i.toRotationMatrix()*vimotion->T_i_c.rotation_matrix();
                SE3    T_w_c(R_w_c,t_w_c);
                curr_frame->T_c_w=T_w_c.inverse();//Tcw = (Twc)^-1
                Vec3 rpy_imu = Q2rpy(q_init_w_i);
                cout << "Init pose by IMU, rpy:" << (57.0*rpy_imu).transpose() << endl;
                Vec3 rpy_cam = Q2rpy(T_w_c.unit_quaternion());
                cout << "Init pose of cam, rpy:" << (57.0*rpy_cam).transpose() << endl;
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
        //STEP1: Recover from LocalMap Feedback msg
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

        //STEP2: Track from frame to frame
        SE3  imu_guess;
        bool has_imu_guess=false;
        if(this->has_imu)
            has_imu_guess = this->vimotion->viGetCorrFrameState(time,imu_guess);
        bool tracking_success;
        tracking_success = lkorb_tracker->tracking(*last_frame,
                                                   *curr_frame,
                                                   imu_guess,
                                                   has_imu_guess,
                                                   K0_rect,
                                                   D0_rect,
                                                   curr_frame->flow_last,
                                                   curr_frame->flow_curr,
                                                   curr_frame->tracking_outlier);
        static int continus_tracking_fail_cnt = 0;
        if(!tracking_success)
        {//cv tracking failed
            continus_tracking_fail_cnt++;
            cout << "[Critical Warning] Tracking Fail-no enough lm pairs" << endl;
            last_frame.swap(curr_frame);//dummy swap, escape this frame
            if(continus_tracking_fail_cnt>=2)
            {
                vo_tracking_state = TrackingFail;
                cout << "Tracking failed! Swith to tracking Fail Mode" << endl;
                continus_tracking_fail_cnt = 0;
            }
            break;
        }
        continus_tracking_fail_cnt = 0;

        //(Option) ->IMU roll pitch compensation
        if(this->has_imu)
        {
            vimotion->viVisionRPCompensation(curr_frame->frame_time, curr_frame->T_c_w);
        }

        //STEP3: In frame BA, remove outliers
        OptimizeInFrame::optimize(*curr_frame);
        vector<Vec2> outlier_reproject;
        double mean_reprojection_error;
        curr_frame->calReprjInlierOutlier(mean_reprojection_error,outlier_reproject,1.5);
        curr_frame->reprojection_error=mean_reprojection_error;
        curr_frame->eraseReprjOutlier();
        //cout << "avg reproj err: " << mean_reprojection_error << endl;

        //(Option) ->IMU state update from vision
        if(this->has_imu)
        {
            vimotion->viCorrectionFromVision(curr_frame->frame_time,
                                             curr_frame->T_c_w,
                                             last_frame->frame_time,
                                             curr_frame->T_c_w,
                                             mean_reprojection_error);
        }

        //STEP5: Redetect
        vector<cv::Point2f> pts2d,pts2d_undistort;
        int newPtsCount;
        int orig_size = curr_frame->landmarks.size();

        this->feature_dem->redetect(curr_frame->img0, curr_frame->get2dPlaneVec(),  pts2d,newPtsCount);
        switch(this->cam_type)
        {
        case DEPTH_D435:
        {
            pts2d_undistort = pts2d;
            break;
        }
        case STEREO_EuRoC_MAV:
        {
            cv::undistortPoints(pts2d,pts2d_undistort,
                                d_camera.K0,d_camera.D0,d_camera.R0,d_camera.P0);
            break;
        }
        }
        bool add_as_inliers=false;
        if(orig_size<60)
        {
            add_as_inliers =true;
        }
        for(size_t i=0; i<pts2d.size(); i++)
        {
            curr_frame->landmarks.push_back(LandMarkInFrame(Vec2(pts2d.at(i).x,
                                                                 pts2d.at(i).y),
                                                            Vec2(pts2d_undistort.at(i).x,
                                                                 pts2d_undistort.at(i).y),
                                                            Vec3(0,0,0),
                                                            false,
                                                            curr_frame->T_c_w,
                                                            add_as_inliers));
        }

        //STEP6: Depth Innovation and Update Landmarks(IIR)
        bool applyiir=true;
        if(mean_reprojection_error>1.0)
            applyiir = false;
        curr_frame->depthInnovation(applyiir);
        curr_frame->eraseNoDepthPoint();

        //STEP7: Record Pose
        ID_POSE tmp;
        tmp.frame_id = curr_frame->frame_id;
        tmp.T_c_w = curr_frame->T_c_w;
        pose_records.push_back(tmp);
        if(pose_records.size() >= 1000)
        {
            pose_records.pop_front();
        }
        //STEP8: Switch KeyFrame if needed
        SE3 T_diff_key_curr = T_c_w_last_keyframe*(curr_frame->T_c_w.inverse());
        Vec3 t=T_diff_key_curr.translation();
        Vec3 r=T_diff_key_curr.so3().log();
        double t_norm = fabs(t[0]) + fabs(t[1]) + fabs(t[2]);
        double r_norm = fabs(r[0]) + fabs(r[1]) + fabs(r[2]);

        if(frameCount<40 && (frameCount%3)==0)
        {
            new_keyframe = true;
            T_c_w_last_keyframe = curr_frame->T_c_w;
        }
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
        if((cnt%3)==0)
        {
            cout << "vision tracking fail, IMU motion only" << endl << "Tring to recover~" << endl;
            if(this->vimotion->viGetCorrFrameState(curr_frame->frame_time,curr_frame->T_c_w))
            {
                if(this->init_frame())
                {
                    new_keyframe = true;
                    vo_tracking_state = Tracking;
                    cout << "Recover succeed" << endl;
                }
                else
                {
                    last_frame.swap(curr_frame);
                    cout << "Recover failure" << endl;
                }
            }
            else
            {
                last_frame.swap(curr_frame);
                cout << "Re-initialization fail: can not find the frame in motion module" << endl;
            }
            cnt=0;
        }
        else
        {
            last_frame.swap(curr_frame);
            if((cnt%2)==0)
            {
                reset_cmd = true;
            }
        }
        break;
    }//end of state: TrackingFail
    }//end of state machine

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    curr_frame->solving_time = (duration.count()/1000.0);
}

bool F2FTracking::init_frame()
{
    bool init_succeed=false;
    vector<cv::Point2f> pts2d;
    this->feature_dem->detect(curr_frame->img0,pts2d);
    cout << "Detect " << pts2d.size() << " Features for init process"<< endl;
    switch (this->cam_type)
    {
    case DEPTH_D435:
    {
        for(size_t i=0; i<pts2d.size(); i++)
        {
            curr_frame->landmarks.push_back(LandMarkInFrame(Vec2(pts2d.at(i).x,
                                                                 pts2d.at(i).y),
                                                            Vec2(pts2d.at(i).x,
                                                                 pts2d.at(i).y),
                                                            Vec3(0,0,0),
                                                            false,
                                                            curr_frame->T_c_w));
        }
        break;
    }
    case STEREO_EuRoC_MAV:{
        vector<cv::Point2f> pts2d_undistort;
        cv::undistortPoints(pts2d,pts2d_undistort,
                            d_camera.K0,d_camera.D0,d_camera.R0,d_camera.P0);
        for(size_t i=0; i<pts2d.size(); i++)
        {
            curr_frame->landmarks.push_back(LandMarkInFrame(Vec2(pts2d.at(i).x,
                                                                 pts2d.at(i).y),
                                                            Vec2(pts2d_undistort.at(i).x,
                                                                 pts2d_undistort.at(i).y),
                                                            Vec3(0,0,0),
                                                            false,
                                                            curr_frame->T_c_w));
        }
        break;
    }
    }
    curr_frame->depthInnovation();
    curr_frame->eraseNoDepthPoint();
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
    return init_succeed;
}

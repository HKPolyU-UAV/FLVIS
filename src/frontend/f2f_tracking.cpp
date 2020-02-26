#include "include/f2f_tracking.h"
#include <chrono>
using namespace std::chrono;

void F2FTracking::init(const int w, const int h,
                       const double cam0_fx_in, const double cam0_fY_in,
                       const double cam0_cx_in, const double cam0_cy_in,
                       const SE3 T_i_c0_in,
                       const TYPEOFCAMERA cam_type_in,
                       const double cam_scale_in,
                       const double cam1_fx_in, const double cam1_fy_in,
                       const double cam1_cx_in, const double cam1_cy_in,
                       const SE3 T_cam0_cam1)
{
    this->feature_dem   = new FeatureDEM(w,h,5);
    this->lkorb_tracker = new LKORBTracking(w,h);
    this->vimotion      = new VIMOTION(T_i_c0_in);
    curr_frame = std::make_shared<CameraFrame>();
    last_frame = std::make_shared<CameraFrame>();
    curr_frame->height = last_frame->height = h;
    curr_frame->width = last_frame->width = w;
    if(cam_type_in==DEPTH_D435I)
    {
        cameraMatrix = (cv::Mat1d(3, 3) << cam0_fx_in, 0, cam0_cx_in, 0, cam0_fY_in, cam0_cy_in, 0, 0, 1);
        distCoeffs   = (cv::Mat1d(4, 1) << 0,0,0,0);
        DepthCamera dc;
        dc.setDepthCamInfo(cam0_fx_in,cam0_fY_in,cam0_cx_in,cam0_cy_in,1000.0);
        curr_frame->d_camera = last_frame->d_camera = dc;
    }
    if(cam_type_in==STEREO_EuRoC_MAV)
    {

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
    }
}

void F2FTracking::correction_feed(const double time, const CorrectionInfStruct corr)
{
    this->correction_inf = corr;
    this->has_localmap_feedback = true;
}

void F2FTracking::image_feed(const double time,
                             const cv::Mat img0,
                             const cv::Mat img1,
                             bool &new_keyframe,
                             bool &reset_cmd)
{
    auto start = high_resolution_clock::now();
    new_keyframe = false;
    reset_cmd = false;
    frameCount++;
    if(frameCount<30) return;
    //15hz Frame Rate
    //if((frameCount%2)==0) return;
    last_frame.swap(curr_frame);
    curr_frame->clear();
    curr_frame->frame_id = frameCount;

    curr_frame->img=img0;
    curr_frame->d_img=img1;
    curr_frame->frame_time = time;

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
                cout << "use imu pose" << endl;
                cout << "roll:"   << rpy[0]*57.2958
                     << " pitch:" << rpy[1]*57.2958
                     << " yaw:"   << rpy[2]*57.2958 << endl;
                Mat3x3 R_w_c = q_init_w_i.toRotationMatrix()*vimotion->T_i_c.rotation_matrix();
                SE3    T_w_c(R_w_c,t_w_c);
                curr_frame->T_c_w=T_w_c.inverse();//Tcw = (Twc)^-1
            }else break;
        }
        vector<Vec2> pts2d;
        vector<cv::Mat>  descriptors;
        this->feature_dem->detect(curr_frame->img,pts2d,descriptors);
        cout << "Detect " << pts2d.size() << " Features"<< endl;
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
            cout << "vo_tracking_state = Tracking" << endl;
        }else
        {
            cout << "No enough measurement for init process" << endl;
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
        solvePnPRansac(p3d,p2d,cameraMatrix,distCoeffs,r_,t_,false,100,3.0,0.99,inliers,cv::SOLVEPNP_P3P);
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
            vimotion->viVisionRPCompensation(curr_frame->frame_time,
                                             curr_frame->T_c_w,
                                             0.05);
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
                                             curr_frame->T_c_w,Vec3(0,0,0));
        }
        //STEP5:
        vector<Vec2> newKeyPts;
        vector<cv::Mat>  newDescriptor;
        int newPtsCount;
        this->feature_dem->redetect(curr_frame->img,
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
        if(t_norm>=0.2 || r_norm>=0.3)
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
            this->feature_dem->detect(curr_frame->img,pts2d,descriptors);
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

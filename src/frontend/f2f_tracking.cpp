#include "include/f2f_tracking.h"
#include <chrono>


void F2FTracking::init(const DepthCamera dc_in,
                       const SE3 T_i_c0_in,
                       const Vec6 feature_para,
                       const Vec6 vi_para,
                       const Vec3 dc_para,
                       const int skip_first_n_imgs_in,
                       const bool need_equal_hist_in
                       )
{
  this->skip_n_imgs = skip_first_n_imgs_in;
  this->need_equal_hist = need_equal_hist_in;
  this->feature_dem   = new FeatureDEM(dc_in.img_w,dc_in.img_h,feature_para);
  this->lkorb_tracker = new LKORBTracking(dc_in.img_w,dc_in.img_h);
  this->vimotion      = new VIMOTION(T_i_c0_in,  9.81,
                                     vi_para[0], vi_para[1],  vi_para[2], vi_para[3]);
  curr_frame = std::make_shared<CameraFrame>();
  last_frame = std::make_shared<CameraFrame>();
  this->cam_type = dc_in.cam_type;
  d_camera = lkorb_tracker->d_camera = curr_frame->d_camera = last_frame->d_camera = dc_in;

  this->iir_ratio = static_cast<float>(dc_para(0));
  this->range = static_cast<float>(dc_para(1));
  if(dc_para(2)<0.5)
  {
      this->enable_dummy = false;
  }else
  {
      this->enable_dummy = true;
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
  cv::Mat img0_out;
  cv::Mat img1_out;
  
  if(img0_in.channels() == 1){
    img0_out = img0_in;
  }
  if(img1_in.channels() == 1){
    img1_out = img1_in;
  }
  if(img0_in.channels()==3)
  {
    if(mbRGB)
      cvtColor(img0_in,img0_out,CV_RGB2GRAY);
    else
      cvtColor(img0_in,img0_out,CV_BGR2GRAY);
  }
  else if(img0_in.channels()==4)
  {
    if(mbRGB)
      cvtColor(img0_in,img0_out,CV_RGBA2GRAY);
    else
      cvtColor(img0_in,img0_out,CV_BGRA2GRAY);
  }
  if(img1_in.channels()==3)
  {
    if(mbRGB)
      cvtColor(img1_in,img1_out,CV_RGB2GRAY);
    else
      cvtColor(img1_in,img1_out,CV_BGR2GRAY);
  }
  else if(img1_in.channels()==4)
  {
    if(mbRGB)
      cvtColor(img1_in,img1_out,CV_RGBA2GRAY);
    else
      cvtColor(img1_in,img1_out,CV_BGRA2GRAY);
  }


  switch(this->cam_type)
  {
  case DEPTH_D435:
  {
    curr_frame->img0=img0_out;
    curr_frame->d_img=img1_out;
    if(skip_n_imgs>0)
    {
      skip_n_imgs--;
      return;
    }
    if(need_equal_hist)
    {
      cv::equalizeHist(curr_frame->img0,curr_frame->img0);
    }
    break;
  }
  case STEREO_RECT:
  case STEREO_UNRECT:
  {
    curr_frame->img0=img0_out;
    curr_frame->img1=img1_out;
    if(skip_n_imgs>0)
    {
      skip_n_imgs--;
      return;
    }
    if(need_equal_hist)
    {
      cv::equalizeHist(curr_frame->img0,curr_frame->img0);
      cv::equalizeHist(curr_frame->img1,curr_frame->img1);
    }
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
    //R_w_c << 1, 0, 0, 0, 1, 0, 0, 0, 1;//If you want to submit it to kitti please uncommen this line
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
    bool optimize_success = OptimizeInFrame::optimize(*curr_frame);
    if(!optimize_success)
    {
      continus_tracking_fail_cnt++;
      cout << "[Critical Warning] Tracking Fail-PnP estimation error!" << endl;
      last_frame.swap(curr_frame);//dummy swap, escape this frame
      if(continus_tracking_fail_cnt>=2)
      {
        vo_tracking_state = TrackingFail;
        cout << "Tracking failed! Swith to tracking Fail Mode" << endl;
        continus_tracking_fail_cnt = 0;
      }
      break;
    }

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
                                       last_frame->T_c_w,
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
      pts2d_undistort = pts2d;
      break;
    case STEREO_RECT:
      pts2d_undistort = pts2d;
      break;
    case STEREO_UNRECT:
      cv::undistortPoints(pts2d,pts2d_undistort,
                          d_camera.K0,d_camera.D0,d_camera.R0,d_camera.P0);
      break;
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
//    bool applyiir=true;
//    if(mean_reprojection_error>1.0)
//      applyiir = false;
    curr_frame->depthInnovation(this->iir_ratio,this->range,this->enable_dummy);
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

    if(frameCount<40 && (frameCount%5)==0)
    {
      new_keyframe = true;
      T_c_w_last_keyframe = curr_frame->T_c_w;
    }
    if(t_norm>=0.05 || r_norm>=0.2)
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
  case STEREO_RECT:
  case STEREO_UNRECT:
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
  curr_frame->depthInnovation(this->iir_ratio,this->range,this->enable_dummy);
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

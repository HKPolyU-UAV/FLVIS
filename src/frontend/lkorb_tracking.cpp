#include "include/lkorb_tracking.h"

LKORBTracking::LKORBTracking(int width_in,int height_in)
{
    this->width=width_in;
    this->height=height_in;
}

bool LKORBTracking::tracking(CameraFrame& from,
                             CameraFrame& to,
                             SE3 T_c_w_guess,
                             bool use_guess,
                             cv::Mat K0_rect,//cam0 rectified cameraMatrix;
                             cv::Mat D0_rect,//cam0 rectified distCoeffs;
                             vector<cv::Point2f>& lm2d_from,
                             vector<cv::Point2f>& lm2d_to,
                             vector<cv::Point2f>& outlier)
{
    lm2d_from.clear();
    lm2d_to.clear();
    outlier.clear();
    //STEP1: Optical Flow
    //STEP2: F matrix check
    //STEP3: PNP_RANSAC
    bool ret=false;

    //STEP1
    vector<int>         in_frame_idx;
    vector<cv::Point2f> from_p2d_plane;
    vector<cv::Point2f> tracked_p2d_plane;
    vector<cv::Point2f> from_p2d_undistort;
    vector<cv::Point2f> tracked_p2d_undistort;
    vector<cv::Point3f> from_p3d;
    from.getAll2dPlaneUndistort3d_cvPf(from_p2d_plane,from_p2d_undistort,from_p3d);
    tracked_p2d_plane = from_p2d_plane;

    vector<unsigned char> mask_tracked;
    vector<float>         err;

    if(use_guess){//project 3d lms to 2d using guess
        switch(d_camera.cam_type)
        {
        case DEPTH_D435:
            for(size_t i=0; i<from_p2d_plane.size(); i++){
                Vec3 lm3d_w(from_p3d.at(i).x,from_p3d.at(i).y,from_p3d.at(i).z);
                Vec3 lm3d_c = DepthCamera::world2cameraT_c_w(lm3d_w,T_c_w_guess);
                Vec2 reProj=from.d_camera.camera2pixel(lm3d_c,
                                                       from.d_camera.cam0_fx,
                                                       from.d_camera.cam0_fy,
                                                       from.d_camera.cam0_cx,
                                                       from.d_camera.cam0_cy);
                tracked_p2d_plane.at(i) = cv::Point2f(reProj[0],reProj[1]);
            }
            break;
        case STEREO_EuRoC_MAV:
            vector<cv::Point2f> project_to_to_img0_plane;
            cv::Mat r_,t_;
            SE3_to_rvec_tvec(T_c_w_guess,r_,t_);
            cv::projectPoints(from_p3d,r_,t_,d_camera.K0,d_camera.D0,project_to_to_img0_plane);
            for(size_t i=0; i<from_p2d_plane.size(); i++){
                tracked_p2d_plane.at(i) = project_to_to_img0_plane.at(i);
            }
            break;
        }
        cv::calcOpticalFlowPyrLK(from.img0, to.img0, from_p2d_plane, tracked_p2d_plane,
                                 mask_tracked, err, cv::Size(21,21), 5,
                                 cv::TermCriteria((cv::TermCriteria::COUNT)+(cv::TermCriteria::EPS), 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
    }else
    {
        cv::calcOpticalFlowPyrLK(from.img0, to.img0, from_p2d_plane, tracked_p2d_plane,
                                 mask_tracked, err, cv::Size(31,31), 5,
                                 cv::TermCriteria((cv::TermCriteria::COUNT)+(cv::TermCriteria::EPS), 30, 0.01),
                                 0);
    }

    switch(d_camera.cam_type)
    {
    case DEPTH_D435:
        from_p2d_undistort = from_p2d_plane;
        tracked_p2d_undistort = tracked_p2d_plane;
        break;
    case STEREO_EuRoC_MAV:
        cv::undistortPoints(tracked_p2d_plane,tracked_p2d_undistort,
                            d_camera.K0,d_camera.D0,d_camera.R0,d_camera.P0);
        break;
    }


    //Creat new frame with all successful Optical Flow result
    to.landmarks.clear();
    int w=to.width-1;
    int h=to.height-1;
    int of_inlier_cnt=0;
    for(int i=from.landmarks.size()-1; i>=0; i--)
    {
        if(mask_tracked.at(i)==1
                &&tracked_p2d_plane.at(i).x>0 && tracked_p2d_plane.at(i).y>0
                &&tracked_p2d_plane.at(i).x<w && tracked_p2d_plane.at(i).y<h)
        {
            of_inlier_cnt++;
            LandMarkInFrame lm=from.landmarks.at(i);
            lm.lm_2d_plane=Vec2(tracked_p2d_plane.at(i).x,tracked_p2d_plane.at(i).y);
            lm.lm_2d_undistort=Vec2(tracked_p2d_undistort.at(i).x,tracked_p2d_undistort.at(i).y);
            to.landmarks.push_back(lm);
        }
        else
        {
            outlier.push_back(from_p2d_plane.at(i));
            from_p2d_plane.erase(from_p2d_plane.begin()+i);
            from_p3d.erase(from_p3d.begin()+i);
            tracked_p2d_plane.erase(tracked_p2d_plane.begin()+i);
            from_p2d_undistort.erase(from_p2d_undistort.begin()+i);
            tracked_p2d_undistort.erase(tracked_p2d_undistort.begin()+i);
        }
    }
    //cout << "of_inlier_cnt: " << of_inlier_cnt << endl;
    if(of_inlier_cnt < 10)
    {
        ret = false;
        return ret;
    }
    //The date below are aligned:
    //to.landmarks
    //from_cvP2f
    //from_cvP3f
    //tracked_cvP2f

    //STEP2 F matrix check
    vector<unsigned char> mask_F_consistant;
    cv::findFundamentalMat(from_p2d_undistort, tracked_p2d_undistort,
                           cv::FM_RANSAC, 3.0, 0.99, mask_F_consistant);
    //F inconsistance point are mark as outlier
    int F_inlier_cnt=0;
    for(int i = 0; i < mask_F_consistant.size(); i++)
    {
        if(mask_F_consistant.at(i)==0)
        {
            outlier.push_back(from_p2d_plane.at(i));
            to.landmarks.at(i).is_tracking_inlier=false;
        }else
        {
            lm2d_from.push_back(from_p2d_plane.at(i));
            lm2d_to.push_back(tracked_p2d_plane.at(i));
        }
    }
    for(LandMarkInFrame lm : to.landmarks)
    {
        if(lm.is_tracking_inlier==true)
            F_inlier_cnt++;
    }
    if(F_inlier_cnt<10){
        ret=false;
        return ret;
    }
    //cout << "F_inlier_cnt: " <<F_inlier_cnt << endl;
    //STEP 3
    vector<unsigned char> mask_pnp_ransac;
    cv::Mat r_ = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat t_ = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat inliers;
    vector<cv::Point2f> p2d;
    vector<cv::Point3f> p3d;
    to.get2dUndistort3dInlierPair_cvPf(p2d,p3d);
    if(use_guess){
        SE3_to_rvec_tvec(T_c_w_guess, r_ , t_ );
        cv::solvePnPRansac(p3d,p2d,K0_rect,D0_rect,
                           r_,t_,true,100,3.0,0.99,inliers,cv::SOLVEPNP_ITERATIVE);
    }else{
        cv::solvePnPRansac(p3d,p2d,K0_rect,D0_rect,
                           r_,t_,false,100,3.0,0.99,inliers,cv::SOLVEPNP_P3P);
    }
    //inlier masks
    int pnp_inlier_cnt=0;
    for (int i = 0; i < (int)p2d.size(); i++){
        mask_pnp_ransac.push_back(0);
    }
    for( int i = 0; i < inliers.rows; i++){
        int n = inliers.at<int>(i);
        mask_pnp_ransac[n] = 1;
        pnp_inlier_cnt++;
    }
    to.updateLMState(mask_pnp_ransac);
    to.T_c_w = SE3_from_rvec_tvec(r_,t_);
    //cout << "pnp_inlier_cnt: " << pnp_inlier_cnt << endl;
    cout << "tracking: " << pnp_inlier_cnt << "|" << F_inlier_cnt << "|" << of_inlier_cnt << endl;
    if(inliers.rows<10)
    {
        ret = false;
        return ret;
    }else
    {
        ret = true;
        return ret;
    }

}

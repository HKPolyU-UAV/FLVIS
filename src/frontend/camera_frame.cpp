#include <include/camera_frame.h>

CameraFrame::CameraFrame()
{
    this->solving_time = 1;
}

void CameraFrame::clear()
{
    T_c_w = SE3(Quaterniond(1,0,0,0),Vec3(0,0,0));
    img0.release();
    img1.release();
    d_img.release();
    landmarks.clear();
}


void CameraFrame::eraseReprjOutlier()
{
    for(int i=this->landmarks.size()-1; i>=0; i--)
    {
        LandMarkInFrame lm=landmarks.at(i);
        if(lm.is_tracking_inlier==false)
        {
            landmarks.erase(landmarks.begin()+i);
        }
    }
}

void CameraFrame::eraseNoDepthPoint()
{
    for(int i=this->landmarks.size()-1; i>=0; i--)
    {
        LandMarkInFrame lm=landmarks.at(i);
        if(lm.has_3d==false)
        {
            landmarks.erase(landmarks.begin()+i);
        }
    }
}


void CameraFrame::calReprjInlierOutlier(double &mean_prjerr, vector<Vec2> &outlier, double sh_over_med)
{
    vector<double> distances;
    vector<double> valid_distances;
    for(LandMarkInFrame &lm :landmarks)
    {
        Vec3 lm3d_w=lm.lm_3d_w;
        Vec3 lm3d_c = DepthCamera::world2cameraT_c_w(lm3d_w,this->T_c_w);
        Vec2 reProj=this->d_camera.camera2pixel(lm3d_c);
        Vec2 lm2d = lm.lm_2d_undistort;
        Vec2 err=lm2d-reProj;
        double distance = err.norm();
        //double relativeDist = sqrt(pow(lm2d(0)-reProj(0),2)+pow(lm2d(1)-reProj(1),2));
        distances.push_back(distance);
        if(distance<5.0){
            valid_distances.push_back(distance);
        }
    }

    //mean SH
    double sum=0;
    for (double &valid_dis : valid_distances)
    {
        sum+=valid_dis;
    }
    mean_prjerr = sum/(double)valid_distances.size();
    //mean SH
    //double sh = mean_prjerr*sh_over_mean;
    //robust MAD SH MAD=1.4826*MED
    sort(valid_distances.begin(), valid_distances.end());
    int half=floor(valid_distances.size()/2);
    double sh = sh_over_med * valid_distances.at(half);

    if(sh>=5.0) sh=5.0;
    //    if(sh<=3.0) sh=3.0;
    for(int i=this->landmarks.size()-1; i>=0; i--)
    {
        if(distances.at(i)>sh)
        {
            outlier.push_back(this->landmarks.at(i).lm_2d_plane);
            this->landmarks.at(i).is_tracking_inlier=false;
        }else
        {
            this->landmarks.at(i).is_tracking_inlier=true;
        }
    }
}

void CameraFrame::recover3DPts_c_FromStereo(vector<Vec3> &pt3ds,
                                            vector<bool> &maskHas3DInf)
{
    pt3ds.clear();
    maskHas3DInf.clear();

    vector<float>   err;
    vector<unsigned char> status;
    vector<cv::Point2f> pt0_plane;
    vector<cv::Point2f> pt2d_0_plane;
    vector<cv::Point2f> pt2d_1_plane;
    vector<cv::Point2f> pt2d_0_undistort;
    vector<cv::Point2f> pt2d_1_undistort;
    vector<cv::Point3f> pt3d;
    this->getAll2dPlaneUndistort3d_cvPf(pt2d_0_plane,pt2d_0_undistort,pt3d);

    pt2d_1_plane = pt2d_0_plane;


    vector<cv::Point2f> project_to_to_img1_plane;
    cv::Mat r_,t_;
    SE3_to_rvec_tvec(d_camera.T_cam1_cam0*T_c_w,r_,t_);
    cv::projectPoints(pt3d,r_,t_,d_camera.K1,d_camera.D1,project_to_to_img1_plane);
    for(int i=0; i<this->landmarks.size(); i++)
    {   //reporject lm to cam1
        if(landmarks.at(i).hasDepthInf()){
            pt2d_1_plane.at(i) = project_to_to_img1_plane.at(i);
        }
    }

    cv::calcOpticalFlowPyrLK(this->img0, this->img1,
                             pt2d_0_plane, pt2d_1_plane,
                             status, err, cv::Size(31,31),5,
                             cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);

    cv::undistortPoints(pt2d_1_plane,pt2d_1_undistort,
                        d_camera.K1,d_camera.D1,d_camera.R1,d_camera.P1);

    for(size_t i=0; i<status.size(); i++)
    {
        if(status.at(i)==1)
        {
            Vec3 pt3d_c;
            if(Triangulation::trignaulationPtFromStereo(Vec2(pt2d_0_undistort.at(i).x,pt2d_0_undistort.at(i).y),
                                                        Vec2(pt2d_1_undistort.at(i).x,pt2d_1_undistort.at(i).y),
                                                        this->d_camera.P0_,
                                                        this->d_camera.P1_,
                                                        pt3d_c))
            {
                pt3ds.push_back(pt3d_c);
                maskHas3DInf.push_back(true);
                continue;
            }
            else
            {
                Vec3 pt3d_c;
                float d_rand;
                d_rand = 0.3 + static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(0.4)));
                pt3d_c = DepthCamera::pixel2camera(Vec2(pt2d_0_undistort.at(i).x,pt2d_0_undistort.at(i).y),
                                                   this->d_camera.cam0_fx,
                                                   this->d_camera.cam0_fy,
                                                   this->d_camera.cam0_cx,
                                                   this->d_camera.cam0_cy,
                                                   d_rand);
                pt3ds.push_back(pt3d_c);
                maskHas3DInf.push_back(false);
                continue;
            }
        }else
        {
            Vec3 pt3d_c;
            float d_rand;
            d_rand = 0.3 + static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(0.4)));
            pt3d_c = DepthCamera::pixel2camera(Vec2(pt2d_0_undistort.at(i).x,pt2d_0_undistort.at(i).y),
                                               this->d_camera.cam0_fx,
                                               this->d_camera.cam0_fy,
                                               this->d_camera.cam0_cx,
                                               this->d_camera.cam0_cy,
                                               d_rand);
            pt3ds.push_back(pt3d_c);
            maskHas3DInf.push_back(false);
            continue;
        }
    }
}

void CameraFrame::recover3DPts_c_FromDepthImg(vector<Vec3>& pt3ds,
                                              vector<bool>& maskHas3DInf)
{
    pt3ds.clear();
    maskHas3DInf.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        //use round to find the nearst pixel from depth image;
        cv::Point2f pt=cv::Point2f(round(landmarks.at(i).lm_2d_plane[0]),round(landmarks.at(i).lm_2d_plane[1]));
        Vec3 pt3d;
        //CV_16UC1 = Z16 16-Bit unsigned int
        if(isnan(d_img.at<ushort>(pt)))
        {
            pt3ds.push_back(Vec3(0,0,0));
            maskHas3DInf.push_back(false);
        }
        else
        {
            float z = (d_img.at<ushort>(pt))/d_camera.cam_scale_factor;
            if(z>=0.3&&z<=8.0)
            {
                pt3d[2] = z;
                pt3d[0] = (pt.x - d_camera.cam0_cx) * z / d_camera.cam0_fx;
                pt3d[1] = (pt.y - d_camera.cam0_cy) * z / d_camera.cam0_fy;
                pt3ds.push_back(pt3d);
                maskHas3DInf.push_back(true);
            }else
            {
                pt3ds.push_back(Vec3(0,0,0));
                maskHas3DInf.push_back(false);
            }
        }
    }
}

void CameraFrame::recover3DPts_c_FromTriangulation(vector<Vec3> &pt3ds, vector<bool> &maskHas3DInf)
{
    pt3ds.clear();
    maskHas3DInf.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        SE3 T_c_w1=landmarks.at(i).lm_1st_obs_frame_pose;
        Vec3 baseline = T_c_w1.translation()-T_c_w.translation();
        if(baseline.norm()>=0.2)
        {
            Vec3 pt3d_w = Triangulation::triangulationPt(landmarks.at(i).lm_1st_obs_2d,landmarks.at(i).lm_2d_undistort,
                                                         landmarks.at(i).lm_1st_obs_frame_pose,T_c_w,
                                                         d_camera.cam0_fx,
                                                         d_camera.cam0_fy,
                                                         d_camera.cam0_cx,
                                                         d_camera.cam0_cy);
            Vec3 pt3d_c = DepthCamera::world2cameraT_c_w(pt3d_w,T_c_w);
            if(pt3d_c[2]>=0.5 && pt3d_c[2]<=15)
            {
                pt3ds.push_back(pt3d_c);
                maskHas3DInf.push_back(true);
            }
            else
            {
                pt3ds.push_back(Vec3(0,0,0));
                maskHas3DInf.push_back(false);
            }
        }
        else
        {
            pt3ds.push_back(Vec3(0,0,0));
            maskHas3DInf.push_back(false);
        }
    }
}
void CameraFrame::depthInnovation(const bool apply_iir)
{
    vector<Vec3> pts3d_c_cam_measure;
    vector<bool> cam_measure_mask;
    vector<Vec3> pts3d_c_triangulation;
    vector<bool> triangulation_mask;
    this->recover3DPts_c_FromTriangulation(pts3d_c_triangulation,triangulation_mask);

    if(this->d_camera.cam_type==DEPTH_D435){
        this->recover3DPts_c_FromDepthImg(pts3d_c_cam_measure,cam_measure_mask);
    }else if(this->d_camera.cam_type==STEREO_EuRoC_MAV){
        this->recover3DPts_c_FromStereo(pts3d_c_cam_measure,cam_measure_mask);
    }

    for(size_t i=0; i<landmarks.size(); i++){
        Vec3 lm_c_measure;
        if(cam_measure_mask.at(i)==false && triangulation_mask.at(i)==false)
        {
            if(this->d_camera.cam_type==STEREO_EuRoC_MAV)
            {
                if(!landmarks.at(i).hasDepthInf())
                {
                    lm_c_measure = pts3d_c_cam_measure.at(i);
                    Vec3 pt3d_w = DepthCamera::camera2worldT_c_w(lm_c_measure,this->T_c_w);
                    landmarks.at(i).lm_3d_c = lm_c_measure;
                    landmarks.at(i).lm_3d_w = pt3d_w;
                    landmarks.at(i).has_3d = true;
                }
                continue;
            }
        }
        if(cam_measure_mask.at(i)==true)
            lm_c_measure = pts3d_c_cam_measure.at(i);
        else
            lm_c_measure = pts3d_c_triangulation.at(i);

        if(apply_iir && landmarks.at(i).hasDepthInf())
        {
            //transfor to Camera frame
            Vec3 lm_c = DepthCamera::world2cameraT_c_w(landmarks.at(i).lm_3d_w,this->T_c_w);
            //apply IIR Filter
            Vec3 lm_c_update = lm_c*0.9+lm_c_measure*0.1;
            //update to world frame
            landmarks.at(i).lm_3d_c = lm_c_update;
            landmarks.at(i).lm_3d_w = DepthCamera::camera2worldT_c_w(lm_c_update,this->T_c_w);
        }
        else//Do not have position
        {
            Vec3 pt3d_w = DepthCamera::camera2worldT_c_w(lm_c_measure,this->T_c_w);
            landmarks.at(i).lm_3d_c = lm_c_measure;
            landmarks.at(i).lm_3d_w = pt3d_w;
            landmarks.at(i).has_3d = true;
        }
    }
}

void CameraFrame::correctLMP3DWByLMP3DCandT(void)
{
    SE3 T=this->T_c_w;
    for(auto lm:landmarks)
    {
        if(lm.hasDepthInf())
        {
            lm.lm_3d_w = DepthCamera::camera2worldT_c_w(lm.lm_3d_c,T);
        }
    }
}

void CameraFrame::forceCorrectLM3DW(const int &cnt, const vector<int64_t> &ids, const vector<Vec3> &lms_3d)
{
    if(cnt<=0) return;
    for(int i=0; i<cnt; i++)
    {
        int correct_lm_id=ids.at(i);
        for(size_t j=0; j<landmarks.size(); j++)
        {
            if(landmarks.at(j).lm_id==correct_lm_id)
            {
                landmarks.at(j).lm_3d_w=lms_3d.at(i);
                break;
            }
        }
    }
}

void CameraFrame::forceMarkOutlier(const int &cnt, const vector<int64_t> &ids)
{
    if(cnt<=0) return;
    for(int i=0; i<cnt; i++)
    {
        int correct_lm_id=ids.at(i);
        for(size_t j=0; j<landmarks.size(); j++)
        {
            if(landmarks.at(j).lm_id==correct_lm_id)
            {
                //                cout <<"match!" << endl;
                landmarks.at(j).is_tracking_inlier=false;
            }
        }
    }
}

void CameraFrame::markAsKF(void)
{
    for(LandMarkInFrame &lm : landmarks)
    {
        lm.is_belong_to_kf = true;
    }
}

int CameraFrame::coVisKFCnt(void)
{
    int cnt=0;
    for(LandMarkInFrame &lm : landmarks)
    {
        if(lm.is_belong_to_kf == true)
        {
            cnt++;
        }
    }
    return cnt;
}

int CameraFrame::validLMCount()
{
    int ret=0;
    if(landmarks.size()>0)
    {
        for(size_t i=0; i<landmarks.size(); i++)
        {
            if(landmarks.at(i).hasDepthInf()&&(landmarks.at(i).is_tracking_inlier==true))
            {
                ret++;
            }
        }
    }
    return ret;
}

void CameraFrame::get2dUndistort3dInlierPair_cvPf(vector<cv::Point2f> &p2d, vector<cv::Point3f> &p3d)
{
    p2d.clear();
    p3d.clear();
    for(LandMarkInFrame &lm : landmarks)
    {
        if(lm.hasDepthInf() && lm.is_tracking_inlier==true)
        {
            p2d.push_back(cv::Point2f(lm.lm_2d_undistort[0],lm.lm_2d_undistort[1]));
            p3d.push_back(cv::Point3f(lm.lm_3d_w[0],lm.lm_3d_w[1],lm.lm_3d_w[2]));
        }
    }
}

void CameraFrame::getAll2dPlaneUndistort3d_cvPf(vector<cv::Point2f> &p2d_p,
                                                vector<cv::Point2f> &p2d_u,
                                                vector<cv::Point3f> &p3d)
{
    p2d_p.clear();
    p2d_u.clear();
    p3d.clear();
    for(LandMarkInFrame &lm : landmarks)
    {
        p2d_p.push_back(cv::Point2f(lm.lm_2d_plane[0],lm.lm_2d_plane[1]));
        p2d_u.push_back(cv::Point2f(lm.lm_2d_undistort[0],lm.lm_2d_undistort[1]));
        p3d.push_back(cv::Point3f(lm.lm_3d_w[0],lm.lm_3d_w[1],lm.lm_3d_w[2]));

    }
}

void CameraFrame::updateLMState(vector<uchar> status)
{
    int indexLM = 0;
    for(size_t i=0; i<landmarks.size(); i++)
    {
        LandMarkInFrame lm=landmarks.at(i);
        if(lm.hasDepthInf() && lm.is_tracking_inlier==true)
        {
            if(status[indexLM] == 0)
                landmarks[i].is_tracking_inlier = false;
            indexLM += 1;
        }
    }

    //cout<<status.size()<<" compare "<<indexLM<<endl;

}
void CameraFrame::getValidInliersPair(vector<LandMarkInFrame> &lms)
{
    lms.clear();
    for(LandMarkInFrame &lm : landmarks)
    {
        if(lm.hasDepthInf() && lm.is_tracking_inlier==true)
        {
            lms.push_back(lm);
        }
    }
}


vector<Vec3> CameraFrame::getValid3dPts(void)
{
    vector<Vec3> ret;
    for(size_t i=0; i<landmarks.size(); i++)
    {
        LandMarkInFrame lm=landmarks.at(i);
        if(lm.hasDepthInf())
        {
            ret.push_back(lm.lm_3d_w);
        }
    }
    return ret;

}


vector<Vec2> CameraFrame::get2dPlaneVec(void)
{
    vector<Vec2> ret;
    ret.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        ret.push_back(landmarks.at(i).lm_2d_plane);
    }
    return ret;
}

vector<cv::Point2f> CameraFrame::get2dPlaneVec_cvPf(void)
{
    vector<cv::Point2f> ret;
    ret.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        ret.push_back(cv::Point2f(landmarks.at(i).lm_2d_plane(0),
                                  landmarks.at(i).lm_2d_plane(0)));
    }
    return ret;
}


void CameraFrame::getKeyFrameInf(vector<int64_t> &lm_id, vector<Vec2> &lm_2d, vector<Vec3> &lm_3d)
{
    lm_id.clear();
    lm_2d.clear();
    lm_3d.clear();
    for(size_t i=0; i<landmarks.size(); i++)
    {
        LandMarkInFrame lm=landmarks.at(i);
        if(lm.hasDepthInf())
        {
            lm_3d.push_back(lm.lm_3d_w);
            lm_2d.push_back(lm.lm_2d_undistort);
            lm_id.push_back(lm.lm_id);
        }
    }

}

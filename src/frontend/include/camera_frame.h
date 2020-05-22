#ifndef CAMERAFRAME_H
#define CAMERAFRAME_H

#include <include/landmark.h>
#include <include/common.h>
#include <include/depth_camera.h>
#include <include/triangulation.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

class CameraFrame
{
public:
    typedef std::shared_ptr<CameraFrame> Ptr;

    int64_t frame_id;
    double  frame_time;
    cv::Mat img0;
    cv::Mat img1;
    cv::Mat d_img;


    int width;
    int height;
    DepthCamera  d_camera;

    vector<LandMarkInFrame> landmarks;

    //camera_pose
    SE3          T_c_w;//Transform from world to camera
    //result
    double solving_time;
    double reprojection_error;

    vector<cv::Point2f> flow_0,flow_1;
    vector<cv::Point2f> depth_inno_outlier;
    vector<cv::Point2f> flow_last,flow_curr;
    vector<cv::Point2f> tracking_outlier;


    CameraFrame();
    void clear();

    void calReprjInlierOutlier(double &mean_prjerr, vector<Vec2> &outlier, double sh_over_med = 3.0);
    void eraseReprjOutlier();
    void updateLMT_c_w();
    void recover3DPts_c_FromDepthImg(vector<Vec3>& pt3ds,
                                     vector<bool>& maskHas3DInf);
    void recover3DPts_c_FromStereo(vector<Vec3>& pt3ds,
                                   vector<bool>& maskHas3DInf);
    void recover3DPts_c_FromTriangulation(vector<Vec3>& pt3ds,
                                          vector<bool>& maskHas3DInf);
    void depthInnovation(const bool apply_iir=true);
    void eraseNoDepthPoint(void);
    void correctLMP3DWByLMP3DCandT(void);//correct lm_3d_w by lm_3d_w and T_c_w
    void forceCorrectLM3DW(const int& cnt, const vector<int64_t>& ids, const vector<Vec3>& lms_3d);
    void forceMarkOutlier( const int& cnt, const vector<int64_t>& ids);
    void markAsKF(void);
    int  coVisKFCnt(void);

    //outlier from ransac pnp
    void updateLMState(vector<uchar> status);

    //IO
    int  validLMCount(void);
    void getAll2dPlane3dPair_cvPf(vector<cv::Point2f>& p2d,vector<cv::Point3f>& p3d);
    void get2dUndistort3dInlierPair_cvPf(vector<cv::Point2f>& p2d,vector<cv::Point3f>& p3d);
    void getValidInliersPair(vector<LandMarkInFrame> &lms);

    void getKeyFrameInf(vector<int64_t>& lm_id, vector<Vec2>& lm_2d, vector<Vec3>& lm_3d);

    vector<Vec2> get2dPlaneVec(void);
    vector<Vec3> getValid3dPts(void);

private:


};

#endif // CAMERAFRAME_H



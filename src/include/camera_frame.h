#ifndef CAMERAFRAME_H
#define CAMERAFRAME_H

#include <include/landmark_in_frame.h>
#include <include/common.h>
#include <include/depth_camera.h>
#include <include/triangulation.h>
#include <opencv2/opencv.hpp>


class CameraFrame
{
public:
    typedef std::shared_ptr<CameraFrame> Ptr;

    int64_t frame_id;
    Mat img;
    Mat d_img;
    int width;
    int height;

    vector<LandMarkInFrame> landmarks;

    //camera_pose
    DepthCamera  d_camera;
    SE3          T_c_w;//Transform from world to camera



    CameraFrame();
    void clear();

    void trackMatchAndEraseOutlier(Mat& newImage,
                                   vector<Vec2>& lm2d_from,
                                   vector<Vec2>& lm2d_to,
                                   vector<Vec2>& outlier);
    void CalReprjInlierOutlier(double &mean_prjerr, vector<Vec2> &outlier, double sh_over_med = 3.0);
    void updateLMT_c_w();

    void recover3DPts_c_FromDepthImg(vector<Vec3>& pt3ds,
                                     vector<bool>& maskHas3DInf);

    void recover3DPts_c_FromTriangulation(vector<Vec3>& pt3ds,
                                          vector<bool>& maskHas3DInf);

    void depthInnovation(void);

    //IO
    void getValid2d3dPair_cvPf(vector<Point2f>& p2d,vector<Point3f>& p3d);
    void getValidInliersPair(vector<LandMarkInFrame> &lms);
    void unpack(vector<Vec2>& pt2d,
                vector<Mat> & descriptors,
                vector<Vec3>& pt3d,
                vector<unsigned char>& mask3d);
    void getKeyFrameInf(vector<int64_t>& lm_id, vector<Vec2>& lm_2d, vector<Vec3>& lm_3d, vector<Mat> &lm_descriptors);


    vector<Point2f> get2dPtsVec_cvP2f(void);
    vector<Point3f> get3dPtsVec_cvP3f(void);
    vector<Vec2> get2dPtsVec(void);
    vector<Vec3> get3dPtsVec(void);
    vector<Mat>  getDescriptorVec(void);
    vector<Vec3> getValid3dPts(void);

private:


};

#endif // CAMERAFRAME_H



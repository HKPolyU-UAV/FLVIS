#ifndef DEPTHcam0_H
#define DEPTHcam0_H

#include "include/common.h"

enum TYPEOFCAMERA{STEREO_RECT,  //rectified stereo images, suitable for d435 stereo mode, kitti dataset.
                  STEREO_UNRECT,//unrectified stereo images, suitable for euroc mav dataset.
                  DEPTH_D435    //depth images, suitable for d435 depth mode.
                 };
//DepthCamera Class
//Support Z16 Depth Format (RealSense d435, d435i)
//Support Stero Camera
//fx,fy,cx,cy are rectified cam para

class DepthCamera
{
public:
    typedef std::shared_ptr<DepthCamera> Ptr;
    //parameter
    enum TYPEOFCAMERA cam_type;
    //image_size
    int img_w,img_h;
    //camera0
    double cam0_fx, cam0_fy, cam0_cx, cam0_cy;
    Mat3x3 K0_;
    Mat3x4 P0_;
    cv::Mat K0,D0,K0_rect,D0_rect,R0,P0;
    //camera1
    double cam1_fx, cam1_fy, cam1_cx, cam1_cy;
    Mat3x3 K1_;
    Mat3x4 P1_;
    cv::Mat K1,D1,K1_rect,D1_rect,R1,P1;
    //extrinic
    SE3    T_cam0_cam1;
    SE3    T_cam1_cam0;
    //for depth camera
    double cam_scale_factor;

    DepthCamera();
    void setDepthCamInfo(const int w_in, const int h_in,
                         const double fx,
                         const double fy,
                         const double cx,
                         const double cy,
                         const double scale_factor = 1000,
                         const enum TYPEOFCAMERA cam_type_in = DEPTH_D435);
    void setSteroCamInfo(const int w_in, const int h_in,
                         const cv::Mat K0_in,      const cv::Mat D0_in,
                         const cv::Mat K0_rect_in, const cv::Mat D0_rect_in, const cv::Mat R0_in, const cv::Mat P0_in,
                         const cv::Mat K1_in,      const cv::Mat D1_in,
                         const cv::Mat K1_rect_in, const cv::Mat D1_rect_in, const cv::Mat R1_in, const cv::Mat P1_in,
                         const SE3 T_c0_c1_in,
                         const enum TYPEOFCAMERA cam_type_in);
    static Vec3 world2cameraT_c_w( const Vec3& p_w, const SE3& T_c_w );
    static Vec3 camera2worldT_c_w( const Vec3& p_c, const SE3& T_c_w );
    static Vec2 camera2pixel( const Vec3& p_c,
                              const double fx, const double fy,
                              const double cx, const double cy);
    static Vec3 pixel2camera( const Vec2& p_p,
                              const double fx, const double fy,
                              const double cx, const double cy,
                              double depth=1 );

    Vec2 camera2pixel( const Vec3& p_c );
    Vec3 pixel2camera( const Vec2& p_p,
                       double depth=1 );

    Vec3 pixel2worldT_c_w( const Vec2& p_p, const SE3& T_c_w, double depth=1 );
    Vec2 world2pixelT_c_w( const Vec3& p_w, const SE3& T_c_w );




private:

};

#endif // DEPTHcam0_H

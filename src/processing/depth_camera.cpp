#include "include/depth_camera.h"

DepthCamera::DepthCamera()
{}

void DepthCamera::setDepthCamInfo(const int w_in, const int h_in,
                                  const double fx,
                                  const double fy,
                                  const double cx,
                                  const double cy,
                                  const double scale_factor,
                                  const enum TYPEOFCAMERA cam_type_in)
{
    this->img_w = w_in;
    this->img_h = h_in;
    cam0_fx = fx;
    cam0_fy = fy;
    cam0_cx = cx;
    cam0_cy = cy;
    K0_ <<cam0_fx,0,cam0_cx,0,cam0_fy,cam0_cy,0,0,1;
    K0_rect = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    D0_rect = (cv::Mat1d(4, 1) << 0,0,0,0);
    cam_scale_factor = scale_factor;
    cam_type = cam_type_in;
}

void DepthCamera::setSteroCamInfo(const int w_in, const int h_in,
                                  const cv::Mat K0_in,      const cv::Mat D0_in,
                                  const cv::Mat K0_rect_in, const cv::Mat D0_rect_in,
                                  const cv::Mat R0_in, const cv::Mat P0_in,
                                  const cv::Mat K1_in,      const cv::Mat D1_in,
                                  const cv::Mat K1_rect_in, const cv::Mat D1_rect_in,
                                  const cv::Mat R1_in, const cv::Mat P1_in,
                                  const SE3 T_c0_c1_in,
                                  const enum TYPEOFCAMERA cam_type_in)
{
    this->img_w = w_in;
    this->img_h = h_in;
    this->K0=K0_in;
    this->D0=D0_in;
    this->K1=K1_in;
    this->D1=D1_in;
    this->K0_rect=K0_rect_in;
    this->K1_rect=K1_rect_in;
    this->D0_rect=D0_rect_in;
    this->D1_rect=D1_rect_in;
    this->R0=R0_in;
    this->R1=R1_in;
    this->P0=P0_in;
    this->P1=P1_in;
//    cout << "K0" << K0 << endl;
//    cout << "D0" << D0 << endl;
//    cout << "K1" << K1 << endl;
//    cout << "D1" << D0 << endl;
//    cout << "K0_rect" << K0_rect << endl;
//    cout << "K1_rect" << K1_rect << endl;
//    cout << "D0_rect" << D0_rect << endl;
//    cout << "D1_rect" << D1_rect << endl;
//    cout << "R0" << R0 << endl;
//    cout << "P0" << P0 << endl;
//    cout << "R1" << R1 << endl;
//    cout << "P1" << P1 << endl;

    T_cam0_cam1 = T_c0_c1_in;
    T_cam1_cam0 = T_cam0_cam1.inverse();

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<4; j++)
        {
            P0_(i,j) = P0.at<double>(i,j);
            P1_(i,j) = P1.at<double>(i,j);
        }
    }

    cam0_fx = P0_(0,0);
    cam0_fy = P0_(1,1);
    cam0_cx = P0_(0,2);
    cam0_cy = P0_(1,2);

    cam1_fx = P1_(0,0);
    cam1_fy = P1_(1,1);
    cam1_cx = P1_(0,2);
    cam1_cy = P1_(1,2);

    K0_ << cam0_fx,0,cam0_cx,0,cam0_fy,cam0_cy,0,0,1;
    K1_ << cam1_fx,0,cam1_cx,0,cam1_fy,cam1_cy,0,0,1;

    cam_type = cam_type_in;
}

Vec3 DepthCamera::world2cameraT_c_w ( const Vec3& p_w, const SE3& T_c_w )
{
    return T_c_w*p_w;
}

Vec3 DepthCamera::camera2worldT_c_w ( const Vec3& p_c, const SE3& T_c_w )
{
    return T_c_w.inverse() *p_c;
}

Vec2 DepthCamera::camera2pixel( const Vec3& p_c,
                                const double fx, const double fy,
                                const double cx, const double cy)
{
    return Vector2d (
                fx * p_c ( 0,0 ) / p_c ( 2,0 ) + cx,
                fy * p_c ( 1,0 ) / p_c ( 2,0 ) + cy
                );
}

Vec2 DepthCamera::camera2pixel ( const Vec3& p_c )
{
    return Vector2d (
                cam0_fx * p_c ( 0,0 ) / p_c ( 2,0 ) + cam0_cx,
                cam0_fy * p_c ( 1,0 ) / p_c ( 2,0 ) + cam0_cy
                );
}

Vec3 DepthCamera::pixel2camera(const Vec2 &p_p,
                               const double fx, const double fy,
                               const double cx, const double cy,
                               double depth)
{
    return Vector3d (
                ( p_p ( 0,0 )-cx ) *depth/fx,
                ( p_p ( 1,0 )-cy ) *depth/fy,
                depth
                );
}

Vec3 DepthCamera::pixel2camera ( const Vec2& p_p, double depth )
{
    return Vector3d (
                ( p_p ( 0,0 )-cam0_cx ) *depth/cam0_fx,
                ( p_p ( 1,0 )-cam0_cy ) *depth/cam0_fy,
                depth
                );
}

Vec2 DepthCamera::world2pixelT_c_w ( const Vec3& p_w, const SE3& T_c_w )
{
    return camera2pixel ( world2cameraT_c_w ( p_w, T_c_w ) );
}

Vec3 DepthCamera::pixel2worldT_c_w ( const Vec2& p_p, const SE3& T_c_w, double depth )
{
    return camera2worldT_c_w ( pixel2camera ( p_p, depth ), T_c_w );
}




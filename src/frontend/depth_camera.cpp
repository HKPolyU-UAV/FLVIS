#include "include/depth_camera.h"

DepthCamera::DepthCamera()
{}

void DepthCamera::setDepthCamInfo(double fx, double fy, double cx, double cy, double scale_factor)
{
    cam0_fx = fx;
    cam0_fy = fy;
    cam0_cx = cx;
    cam0_cy = cy;
    K0_ <<cam0_fx,0,cam0_cx,0,cam0_fy,cam0_cy,0,0,1;
    cam_scale_factor = scale_factor;
    cam_type = DEPTH_D435;
}

void DepthCamera::setSteroCamInfo(const cv::Mat K0_in, const cv::Mat D0_in, const Mat3x4 P0_in,
                                  const cv::Mat K1_in, const cv::Mat D1_in, const Mat3x4 P1_in,
                                  const SE3 T_c0_c1_in)
{
    this->P0_ = P0_in;
    this->P1_ = P1_in;
    this->K0=K0_in;
    this->K1=K1_in;
    this->D0=D0_in;
    this->D1=D1_in;
    //    cout << "-------------------" << endl;
    //    cout << K0 << endl;
    //    cout << K1 << endl;
    //    cout << D0 << endl;
    //    cout << D1 << endl;
    //    cout << "-------------------" << endl;
    cam0_fx = P0_in(0,0);
    cam0_fy = P0_in(1,1);
    cam0_cx = P0_in(0,2);
    cam0_cy = P0_in(1,2);

    cam1_fx = P1_in(0,0);
    cam1_fy = P1_in(1,1);
    cam1_cx = P1_in(0,2);
    cam1_cy = P1_in(1,2);
    K0_ << cam0_fx,0,cam0_cx,0,cam0_fy,cam0_cy,0,0,1;
    K1_ << cam1_fx,0,cam1_cx,0,cam1_fy,cam1_cy,0,0,1;
    T_cam0_cam1 = T_c0_c1_in;
    T_cam1_cam0 = T_cam0_cam1.inverse();
    cam_type = STEREO_EuRoC_MAV;
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





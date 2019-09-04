#include "include/depth_camera.h"

DepthCamera::DepthCamera()
{}

DepthCamera::DepthCamera(double fx, double fy, double cx, double cy, double scale_factor)
{
  camera_fx = fx;
  camera_fy = fy;
  camera_cx = cx;
  camera_cy = cy;
  camera_scale_factor = scale_factor;
}

void DepthCamera::setCameraInfo(double fx, double fy, double cx, double cy, double scale_factor)
{
  camera_fx = fx;
  camera_fy = fy;
  camera_cx = cx;
  camera_cy = cy;
  camera_scale_factor = scale_factor;
}




Vec3 DepthCamera::world2cameraT_c_w ( const Vec3& p_w, const SE3& T_c_w )
{
    return T_c_w*p_w;
}

Vec3 DepthCamera::camera2worldT_c_w ( const Vec3& p_c, const SE3& T_c_w )
{
    return T_c_w.inverse() *p_c;
}

Vec2 DepthCamera::camera2pixel ( const Vec3& p_c )
{
    return Vector2d (
        camera_fx * p_c ( 0,0 ) / p_c ( 2,0 ) + camera_cx,
        camera_fy * p_c ( 1,0 ) / p_c ( 2,0 ) + camera_cy
    );
}

Vec3 DepthCamera::pixel2camera ( const Vec2& p_p, double depth )
{
    return Vector3d (
        ( p_p ( 0,0 )-camera_cx ) *depth/camera_fx,
        ( p_p ( 1,0 )-camera_cy ) *depth/camera_fy,
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





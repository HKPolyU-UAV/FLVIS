#ifndef DEPTHCAMERA_H
#define DEPTHCAMERA_H

#include "common.h"

//DepthCamera Class
//Support Z16 depth format


class DepthCamera
{
public:
  typedef std::shared_ptr<DepthCamera> Ptr;

  //parameter
  double camera_fx;
  double camera_fy;
  double camera_cx;
  double camera_cy;
  double camera_scale_factor;

  DepthCamera();

  DepthCamera(double fx, double fy, double cx, double cy, double scale_factor=1);

  void setCameraInfo(double fx, double fy, double cx, double cy, double scale_factor=1);

  static Vec3 world2cameraT_c_w( const Vec3& p_w, const SE3& T_c_w );
  static Vec3 camera2worldT_c_w( const Vec3& p_c, const SE3& T_c_w );

  Vec2 camera2pixel( const Vec3& p_c );
  Vec3 pixel2camera( const Vec2& p_p, double depth=1 );

  Vec3 pixel2worldT_c_w( const Vec2& p_p, const SE3& T_c_w, double depth=1 );
  Vec2 world2pixelT_c_w( const Vec3& p_w, const SE3& T_c_w );

private:

};

#endif // DEPTHCAMERA_H

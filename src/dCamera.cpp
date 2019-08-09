#include "include/dCamera.h"

DepthCamera::DepthCamera()
{

}

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

Vec3 DepthCamera::world2camera ( const Vec3& p_w, const SE3& T_c_w )
{
    return T_c_w*p_w;
}

Vec3 DepthCamera::camera2world ( const Vec3& p_c, const SE3& T_c_w )
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

Vec2 DepthCamera::world2pixel ( const Vec3& p_w, const SE3& T_c_w )
{
    return camera2pixel ( world2camera ( p_w, T_c_w ) );
}

Vec3 DepthCamera::pixel2world ( const Vec2& p_p, const SE3& T_c_w, double depth )
{
    return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
}


void DepthCamera::recover3DPtsFromDepthImg(const Mat& dImg,
                                           const vector<Vec2>& pt2ds,
                                           vector<Vec3>& pt3ds,
                                           vector<unsigned char>& maskHas3DInf)
{
  pt3ds.clear();
  maskHas3DInf.clear();
  for(size_t i=0; i<pt2ds.size(); i++)
  {
    //use round to find the nearst pixel from depth image;

    Point2f pt=Point2f(round(pt2ds.at(i)[0]),round(pt2ds.at(i)[1]));
    Vec3 pt3d;
    //CV_16UC1 = Z16 16-Bit unsigned int
    if(isnan(dImg.at<ushort>(pt)))
    {
      pt3ds.push_back(Vec3(0,0,0));
      maskHas3DInf.push_back(0);
    }
    else
    {
      float z = (dImg.at<ushort>(pt))/camera_scale_factor;
      if(z>=0.5&&z<=6.5)
      {
        pt3d[2] = z;
        pt3d[0] = (pt.x - camera_cx) * z / camera_fx;
        pt3d[1] = (pt.y - camera_cy) * z / camera_fy;
        pt3ds.push_back(pt3d);
        maskHas3DInf.push_back(1);
      }else
      {
        pt3ds.push_back(Vec3(0,0,0));
        maskHas3DInf.push_back(0);
      }
    }

  }
}



#include <include/cameraFrame.h>

CameraFrame::CameraFrame()
{

}

void CameraFrame::clear()
{
  T_c_w = SE3(Quaterniond(1,0,0,0),Vec3(0,0,0));
  img.release();
  d_img.release();
  landmarks.clear();
}

void CameraFrame::getValid2d3dPair_cvPf(vector<Point2f> &p2d, vector<Point3f> &p3d)
{
  p2d.clear();
  p3d.clear();
  for(size_t i=0; i<landmarks.size(); i++)
  {
    LandMarkInFrame lm=landmarks.at(i);
    if(lm.hasDepthInf())
    {
      p2d.push_back(Point2f(lm.lmPt2d[0],lm.lmPt2d[1]));
      p3d.push_back(Point3f(lm.lmPt3d[0],lm.lmPt3d[1],lm.lmPt3d[2]));
    }
  }
}

void CameraFrame::updateDepthMeasurement(void)
{
  vector<Vec3> pts3d_c;
  vector<unsigned char> depthMeasureMask;
  this->d_camera.recover3DPtsFromDepthImg(this->d_img,this->get2dPtsVec(),pts3d_c,depthMeasureMask);
  for(size_t i=0; i<this->landmarks.size(); i++)
  {
    if(depthMeasureMask.at(i)==1)
    {
      if(this->landmarks.at(i).hasDepthInf())
      {

      }
      else//Do not have position
      {
        Vec3 pt3d_w = this->d_camera.camera2worldT_c_w(pts3d_c.at(i),this->T_c_w);
        landmarks.at(i).lmPt3d = pt3d_w;
        landmarks.at(i).lmState = LMSTATE_NORMAL;
      }
    }
    else
    {continue;}
  }
}

void CameraFrame::removeOutliers(double sh_reprojection_error)
{
  for(int i=this->landmarks.size()-1; i>=0; i--)
  {
    if(landmarks.at(i).hasDepthInf())
    {
      Vec3 lm3d_w=landmarks.at(i).lmPt3d;
      Vec2 reProj=this->d_camera.world2pixelT_c_w(lm3d_w,this->T_c_w);
      Vec2 lm2d = landmarks.at(i).lmPt2d;
      if(sqrt(pow(lm2d(0)-reProj(0),2)+pow(lm2d(1)-reProj(1),2))>=sh_reprojection_error)
      {
        landmarks.erase(landmarks.begin()+i);
      }
    }
  }
}

void CameraFrame::unpack(vector<Vec2> &pt2d,
                         vector<Mat>  &descriptors,
                         vector<Vec3> &pt3d,
                         vector<unsigned char> &mask3d)
{
  pt2d.clear();
  descriptors.clear();
  pt3d.clear();
  mask3d.clear();
  for(size_t i=0; i<pt2d.size(); i++)
  {
    pt2d.push_back(landmarks.at(i).lmPt2d);
    pt3d.push_back(landmarks.at(i).lmPt3d);
    descriptors.push_back(landmarks.at(i).lmDescriptor);
    if(landmarks.at(i).lmState==LMSTATE_NORMAL || landmarks.at(i).lmState==LMSTATE_CONVERGED)
    {mask3d.push_back(1);}
    else
    {mask3d.push_back(0);}
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
      ret.push_back(lm.lmPt3d);
    }
  }
  return ret;

}

vector<Point2f> CameraFrame::get2dPtsVec_cvP2f(void)
{
  vector<Point2f> ret;
  ret.clear();
  for(size_t i=0; i<landmarks.size(); i++)
  {
    ret.push_back(Point2f(landmarks.at(i).lmPt2d[0],
                  landmarks.at(i).lmPt2d[1]));
  }
  return ret;
}
vector<Point3f> CameraFrame::get3dPtsVec_cvP3f(void)
{
  vector<Point3f> ret;
  ret.clear();
  for(size_t i=0; i<landmarks.size(); i++)
  {
    ret.push_back(Point3f(landmarks.at(i).lmPt2d[0],
                  landmarks.at(i).lmPt2d[1],
        landmarks.at(i).lmPt2d[3]));
  }
  return ret;
}

vector<Vec2> CameraFrame::get2dPtsVec(void)
{
  vector<Vec2> ret;
  ret.clear();
  for(size_t i=0; i<landmarks.size(); i++)
  {
    ret.push_back(landmarks.at(i).lmPt2d);
  }
  return ret;
}

vector<Vec3> CameraFrame::get3dPtsVec(void)
{
  vector<Vec3> ret;
  ret.clear();
  for(size_t i=0; i<landmarks.size(); i++)
  {
    ret.push_back(landmarks.at(i).lmPt3d);
  }
  return ret;
}

vector<Mat>  CameraFrame::getDescriptorVec(void)
{
  vector<Mat> ret;
  ret.clear();
  for(size_t i=0; i<landmarks.size(); i++)
  {
    ret.push_back(landmarks.at(i).lmDescriptor);
  }
  return ret;
}

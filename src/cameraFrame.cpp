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

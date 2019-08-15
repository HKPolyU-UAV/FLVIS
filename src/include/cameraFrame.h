#ifndef CAMERAFRAME_H
#define CAMERAFRAME_H

#include <include/common.h>
#include <include/dCamera.h>
#include <lmInFrame.h>
#include <opencv2/opencv.hpp>

class CameraFrame
{
public:
  typedef std::shared_ptr<CameraFrame> Ptr;


  Mat img;
  Mat d_img;

  vector<LandMarkInFrame> landmarks;

  //camera_pose
  DepthCamera  d_camera;
  SE3          T_c_w;//Transform from world to camera



  CameraFrame();
  void clear();

  void trackMatchAndUpdateLMs(Mat& newImage,
                                     vector<Vec2>& lm2d_from,
                                     vector<Vec2>& lm2d_to,
                                     vector<Vec2>& outlier);
  void removeReprojectionOutliers(vector<Vec2> &outlier,
                                  double SH = 3.0);
  void updateDepthMeasurement(void);

  //IO
  void getValid2d3dPair_cvPf(vector<Point2f>& p2d,vector<Point3f>& p3d);
  void unpack(vector<Vec2>& pt2d,
              vector<Mat> & descriptors,
              vector<Vec3>& pt3d,
              vector<unsigned char>& mask3d);
  vector<Point2f> get2dPtsVec_cvP2f(void);
  vector<Point3f> get3dPtsVec_cvP3f(void);
  vector<Vec2> get2dPtsVec(void);
  vector<Vec3> get3dPtsVec(void);
  vector<Mat>  getDescriptorVec(void);
  vector<Vec3> getValid3dPts(void);

private:


};

#endif // CAMERAFRAME_H



#include <include/camera_frame.h>

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

void CameraFrame::trackMatchAndUpdateLMs(Mat& newImage,
                                         vector<Vec2>& lm2d_from,
                                         vector<Vec2>& lm2d_to,
                                         vector<Vec2>& outlier)
{
  //STEP1: Track Match ORB
  //STEP2: Computer ORB
  //STEP3: Check Matches
  //STEP4: Update the lm2d and output
  //STEP1:
  vector<Point2f> trackedLM_cvP2f;
  vector<float> err;
  vector<unsigned char> mask_tracked;
  TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
  calcOpticalFlowPyrLK(img, newImage,
                       this->get2dPtsVec_cvP2f(), trackedLM_cvP2f,
                       mask_tracked, err, Size(31,31), 2, criteria);

  //Compute ORB
  vector<unsigned char> mask_hasDescriptor;
  vector<Mat>           trackedLMDescriptors;
  vector<KeyPoint>      trackedLM_cvKP;
  Mat descriptorsMat;
  KeyPoint::convert(trackedLM_cvP2f,trackedLM_cvKP);
  cv::Ptr<DescriptorExtractor> extractor = ORB::create();
  extractor->compute(newImage, trackedLM_cvKP, descriptorsMat);

  for(size_t i=0; i<trackedLM_cvP2f.size(); i++)
  {
    unsigned char hasDescriptor = 0;
    for(size_t j=0; j<trackedLM_cvKP.size(); j++)
    {
      if(trackedLM_cvP2f.at(i).x==trackedLM_cvKP.at(j).pt.x &&
         trackedLM_cvP2f.at(i).y==trackedLM_cvKP.at(j).pt.y)
      {//has ORB descriptor
        trackedLMDescriptors.push_back(descriptorsMat.row(j));
        hasDescriptor = 1;
        break;
      }
    }
    mask_hasDescriptor.push_back(hasDescriptor);
    if(hasDescriptor==0)
    {
      cv::Mat zeroDescriptor(cv::Size(32, 1), CV_8U, Scalar(0));
      trackedLMDescriptors.push_back(zeroDescriptor);
    }
  }
  //Match Check
  vector<unsigned char> mask_matchCheck;
  for(size_t i=0; i<trackedLMDescriptors.size(); i++)
  {
    if(norm(landmarks.at(i).lmDescriptor, trackedLMDescriptors.at(i), NORM_HAMMING) <= 100)
    {
      mask_matchCheck.push_back(1);
    }else
    {
      mask_matchCheck.push_back(0);
    }
  }

  //STEP4:
  for(int i=landmarks.size()-1; i>=0; i--)
  {
    if(mask_tracked.at(i)!=1       ||
       mask_hasDescriptor.at(i)!=1 ||
       mask_matchCheck.at(i)!=1)
    {//outliers
      outlier.push_back(this->landmarks.at(i).lmPt2d);
      trackedLM_cvP2f.erase(trackedLM_cvP2f.begin()+i);
      this->landmarks.erase(this->landmarks.begin()+i);
    }
  }
  for(size_t i=0; i<landmarks.size(); i++)
  {
    lm2d_from.push_back(landmarks.at(i).lmPt2d);
    lm2d_to.push_back(Vec2(trackedLM_cvP2f.at(i).x,trackedLM_cvP2f.at(i).y));
  }
}

void CameraFrame::removeReprojectionOutliers(vector<Vec2> &outlier,
                                             double SH)
{
  vector<double> vecDistances;
  vector<double> vecValidDistances;
  for(int i=this->landmarks.size()-1; i>=0; i--)
  {
    if(landmarks.at(i).hasDepthInf())//has depth information
    {
      Vec3 lm3d_w=landmarks.at(i).lmPt3d;
      Vec3 lm3d_c = DepthCamera::world2cameraT_c_w(lm3d_w,this->T_c_w);
      Vec2 reProj=this->d_camera.camera2pixel(lm3d_c);
      Vec2 lm2d = landmarks.at(i).lmPt2d;
      double relativeDist = sqrt(pow(lm2d(0)-reProj(0),2)+pow(lm2d(1)-reProj(1),2))/lm3d_c[2];
      vecDistances.push_back(relativeDist);
      vecValidDistances.push_back(relativeDist);
    }
    else//no depth information
    {
      vecDistances.push_back(0);
    }
  }
  //calculate sh using MAD=1.4826*MED Median absolute deviation
//  sort(vecValidDistances.begin(), vecValidDistances.end());
//  int half=floor(vecValidDistances.size()/2);
//  double MAD = MADOverMED*vecValidDistances.at(half);
//  cout << "MAD SH=" << MAD << endl;
  //erase outlier
  for(int i=this->landmarks.size()-1; i>=0; i--)
  {
    if(vecDistances.at(i)>SH)
    {
      outlier.push_back(this->landmarks.at(i).lmPt2d);
      this->landmarks.erase(this->landmarks.begin()+i);
    }
  }
}

void CameraFrame::updateDepthMeasurement(void)
{
  vector<Vec3> pts3d_c;
  vector<unsigned char> depthMeasureMask;
  this->d_camera.recover3DPtsFromDepthImg(this->d_img,this->get2dPtsVec(),pts3d_c,depthMeasureMask);
  for(size_t i=0; i<landmarks.size(); i++)
  {
    if(depthMeasureMask.at(i)==1)
    {
      if(landmarks.at(i).hasDepthInf())
      {
       //transfor to Camera frame
       Vec3 lm_c = DepthCamera::world2cameraT_c_w(landmarks.at(i).lmPt3d,this->T_c_w);
       Vec3 lm_c_measure = pts3d_c.at(i);
       //apply IIR Filter
       Vec3 lm_c_update = lm_c*0.99+lm_c_measure*0.01;
       //update to world frame
       landmarks.at(i).lmPt3d = DepthCamera::camera2worldT_c_w(lm_c_update,this->T_c_w);
       landmarks.at(i).lmState = LMSTATE_NORMAL;
      }
      else//Do not have position
      {
        Vec3 pt3d_w = DepthCamera::camera2worldT_c_w(pts3d_c.at(i),this->T_c_w);
        landmarks.at(i).lmPt3d = pt3d_w;
        landmarks.at(i).lmState = LMSTATE_NORMAL;
      }
    }
    else
    {continue;}
  }
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

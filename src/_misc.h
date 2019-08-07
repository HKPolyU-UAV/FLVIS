#ifndef MISC_H
#define MISC_H

//Draw on the image

#include <include/common_def.h>

//Vec3 cvVec2EigenVec_3(const Mat tvec)
//{
//  Vec3 ret(tvec(0,0),tvec(0,1));
//  ret << R(0,0),R(0,1),R(0,2),R(1,0),R(1,1),R(1,2),R(2,0),R(2,1),R(2,2);
//  return ret;
//}

Mat3x3 cvMat2EigenMatrix_3x3(const Mat R)
{
  Mat3x3 ret;
  ret<<R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
      R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
      R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);
  return ret;
}

Vec3 cvMat2EigenVector_3(const Mat t)
{
  return Vec3(t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0));
}

void ORBDescriptorMat2VecMat(const Mat& descriptorMat, vector<Mat>& vecDescriptorMat)
{
  vecDescriptorMat.clear();
  for(int i=0; i<descriptorMat.size().height;i++)
  {
    vecDescriptorMat.push_back(descriptorMat.row(i));
  }
}

Point2f toCVP2f(Vec2 pt)
{
  return Point2f(pt[0],pt[1]);
}

Point3f toCVP3f(Vec3 pt)
{
  return Point3f(pt[0],pt[1],pt[2]);
}

vector<Point2f> toCVPoint2f(vector<Vec2>& pt2ds)
{
  vector<Point2f> ret;
  ret.clear();
  for(size_t i=0; i<pt2ds.size(); i++)
    ret.push_back(Point2f(pt2ds.at(i)[0],pt2ds.at(i)[1]));
  return ret;
}

void drawRegion16(Mat& img)
{
  int divH = floor(img.size().height/4);
  int divW = floor(img.size().width/4);
  int x,y;
  for(int i=1; i<=3; i++)//horizon
  {
    y=i*divH;
    line(img, Point(0,y), Point((img.size().width-1),y), Scalar(255,255,255),1);//horizon
    x=i*divW;
    line(img, Point(x,0), Point(x,(img.size().height-1)), Scalar(255,255,255),1);//vertical
  }
}

void drawKeyPts(Mat& img, const vector<Point2f>& KeyPts)
{
  for(size_t i=0; i<KeyPts.size(); i++)
  {
    Point pt(floor(KeyPts.at(i).x),floor(KeyPts.at(i).y));
    circle(img, pt, 2, Scalar( 255, 0, 0 ), 3);
  }
}

void drawFlow(Mat& img, const vector<Point2f>& from, const vector<Point2f>& to)
{
  if(from.size()==to.size())
  {
    for(size_t i=0; i<from.size(); i++)
    {
      Point pt_from(floor(from.at(i).x),floor(from.at(i).y));
      Point pt_to(floor(to.at(i).x),floor(to.at(i).y));
      circle(img, pt_from, 2, Scalar( 0, 255, 0 ));
      arrowedLine(img, pt_from, pt_to, Scalar( 0,204,204),2);
    }
  }
}


#endif // MISC_H

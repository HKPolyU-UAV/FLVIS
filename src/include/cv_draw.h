#ifndef CVDRAW_H
#define CVDRAW_H

#include <include/common.h>
#include <include/camera_frame.h>


inline void drawFPS(Mat& img, int fps)
{
    putText(img, "FPS:"+std::to_string(fps), Point(0,20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0), 2, LINE_AA);
}

inline void drawRegion16(Mat& img)
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

inline void drawKeyPts(Mat& img, const vector<Point2f>& KeyPts)
{
  for(size_t i=0; i<KeyPts.size(); i++)
  {
    Point pt(floor(KeyPts.at(i).x),floor(KeyPts.at(i).y));
    circle(img, pt, 2, Scalar( 255, 0, 0 ), 3);
  }
}

inline void drawOutlier(Mat& img, const vector<Vec2>& outlier)
{
  for(size_t i=0; i<outlier.size(); i++)
  {
    Point pt(floor(outlier.at(i)[0]),floor(outlier.at(i)[1]));
    circle(img, pt, 2, Scalar( 255, 255, 255 ), 2);
  }
}

inline void drawFlow(Mat& img, const vector<Vec2>& from, const vector<Vec2>& to)
{
  if(from.size()==to.size())
  {
    for(size_t i=0; i<from.size(); i++)
    {
      Point pt_from(floor(from.at(i)[0]),floor(from.at(i)[1]));
      Point pt_to(floor(to.at(i)[0]),floor(to.at(i)[1]));
      //circle(img, pt_from, 2, Scalar( 0, 255, 0 ));
      arrowedLine(img, pt_from, pt_to, Scalar( 0,204,204),1);
    }
  }
}

inline void drawFrame(Mat& img, CameraFrame& frame)
{
  drawRegion16(img);
  int fps=floor(1/frame.solving_time);
  putText(img, "FPS:"+std::to_string(fps),
          Point(0,20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0), 2, LINE_8);
  std::stringstream stream;
  stream << std::fixed << std::setprecision(2) << frame.reprojection_error;
  putText(img, "ERR:"+stream.str(),
          Point(img.cols-150,20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0), 2, LINE_8);
  for(auto lm:frame.landmarks)
  {
    if(lm.hasDepthInf())
    {
      float z=lm.lm_3d_c[2];
      if(z>=6) z=6;
      if(z<0)  z=0;
      int r=floor(z*42);
      int b=255-r;
      Point pt(round(lm.lm_2d[0]),round(lm.lm_2d[1]));
      circle(img, pt, 2, Scalar( b, 0, r ), 2);
    }
  }
}

#endif // CVDRAW_H

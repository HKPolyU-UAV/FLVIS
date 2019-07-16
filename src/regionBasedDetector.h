#ifndef REGIONBASEDDETECTOR_H
#define REGIONBASEDDETECTOR_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <utility>
/* Region Based FAST Detector
 * Regions:
 *  | 0 | 1 | 2 | 3 |
 *  | 4 | 5 | 6 | 7 |
 *  | 8 | 9 | 10| 11|
 *  | 12| 13| 14| 15|
 *  //Detect FAST features
 *  //Devided all features into 16 regions
 *  //For every region, select features by Harris index and boundary size
 * */
#define MAX_REGION_FREATURES_NUM (6)
#define MIN_REGION_FREATURES_NUM (3)

using namespace cv;
using namespace std;


// Driver function to sort the vector elements by
// second element of pair in descending order
bool sortbysecdesc(const pair<Point2f,float> &a,
                   const pair<Point2f,float> &b)
{
  return a.second>b.second;
}

class regionBasedDetector
{
private:
  int width;
  int height;
  int regionWidth;
  int regionHeight;
  int boundary_dis;
  vector<pair<Point2f,float> > regionKeyPts[16];

  calHarrisR(Mat& img, Point2f& Pt, float &R)
  {
    uchar patch[9];
    int xx = Pt.x;
    int yy = Pt.y;
    patch[0]=img.at<uchar>(cv::Point(xx-1,yy-1));
    patch[1]=img.at<uchar>(cv::Point(xx,yy-1));
    patch[2]=img.at<uchar>(cv::Point(xx+1,yy-1));
    patch[3]=img.at<uchar>(cv::Point(xx-1,yy));
    patch[4]=img.at<uchar>(cv::Point(xx,yy));
    patch[5]=img.at<uchar>(cv::Point(xx+1,yy+1));
    patch[6]=img.at<uchar>(cv::Point(xx-1,yy+1));
    patch[7]=img.at<uchar>(cv::Point(xx,yy+1));
    patch[8]=img.at<uchar>(cv::Point(xx+1,yy+1));
    float IX,IY;
    float X2,Y2;
    float XY;
    IX = (patch[0]+patch[3]+patch[6]-(patch[2]+patch[5]+patch[8]))/3;
    IY = (patch[0]+patch[1]+patch[2]-(patch[6]+patch[7]+patch[8]))/3;
    X2 = IX*IX;
    Y2 = IY*IX;
    XY = IX*IX;
    //M = | X2  XY |
    //    | XY  Y2 |
    //R = det(M)-k(trace^2(M))
    //  = X2*Y2-XY*XY  - 0.05*(X2+Y2)*(X2+Y2)
    R = (X2*Y2)-(XY*XY) - 0.05*(X2+Y2)*(X2+Y2);
  }

  filterAndFillToRegion(Mat& img, vector<Point2f>& pts)
  {
    //Devided all features into 16 regions
    for(size_t i=0; i<pts.size(); i++)
    {
      Point2f pt = pts.at(i);
      if (pt.x>=10 && pt.x<(width-10) && pt.y>=10 && pt.y<(height-10))
      {
        float Harris_R;
        calHarrisR(img,pt,Harris_R);
        if(Harris_R>50.0)
        {
          int regionNum= 4*floor(pt.y/regionHeight) + (pt.x/regionWidth);
          regionKeyPts[regionNum].push_back(make_pair(pt,Harris_R));
        }
      }
    }
    for(int i=0; i<16; i++)
    {
      cout << regionKeyPts[i].size() << "in Region " << i << endl;
    }
  }

public:

  regionBasedDetector(int image_width, int image_height, int boundaryBoxSize=5)
  {
    float a=2;
    width=image_width;
    height=image_height;
    regionWidth  = floor(width/4.0);
    regionHeight = floor(height/4.0);
    boundary_dis = floor(boundaryBoxSize/2.0);
  }
  ~regionBasedDetector();

  refill(Mat& img, vector<Point2f>& outKeyPts)
  {

  }

  detect(Mat& img, vector<Point2f>& outKeyPts)
  {
    Ptr<FastFeatureDetector> detector= FastFeatureDetector::create();
    //Clear
    outKeyPts.clear();
    for(int i=0; i<16; i++){
      regionKeyPts[i].clear();
    }
    //Detect FAST
    vector<KeyPoint> FASTFeatures;
    vector<Point2f>  allKeyPoints;
    detector->detect(img, FASTFeatures);
    cout << FASTFeatures.size() << endl;
    KeyPoint::convert(FASTFeatures,allKeyPoints);


    filterAndFillToRegion(img,allKeyPoints);

    //For every region, select features by Harris index and boundary size
    for(int i=0; i<16; i++)
    {
      sort(regionKeyPts[i].begin(), regionKeyPts[i].end(), sortbysecdesc);
      vector<pair<Point2f,float>> tmp = regionKeyPts[i];
      regionKeyPts[i].clear();
      int count = 0;
      for(size_t j=0; j<tmp.size(); j++)
      {
        int outSideConflictBoundary = 1;
        for(size_t k=0; k<regionKeyPts[i].size(); k++)
        {
          float dis_x = fabs(tmp.at(j).first.x-regionKeyPts[i].at(k).first.x);
          float dis_y = fabs(tmp.at(j).first.y-regionKeyPts[i].at(k).first.y);
          if(dis_x<=boundary_dis || dis_y<=boundary_dis)
          {
            outSideConflictBoundary=0;
          }
        }
        if(outSideConflictBoundary)
        {
          regionKeyPts[i].push_back(tmp.at(j));
          count++;
          if(count>=MAX_REGION_FREATURES_NUM) break;
        }
      }
    }

    for(int i=0; i<16; i++)
    {
      //cout << regionKeyPts[i].size() << "in Region " << i << endl;
      for(size_t j=0; j<regionKeyPts[i].size(); j++)
      {
        outKeyPts.push_back(regionKeyPts[i].at(j).first);
      }
    }
    //    cout << "size "<< outKeyPts.size() << endl;
  }


};//class RegionFeatureDetector

#endif // REGIONBASEDDETECTOR_H

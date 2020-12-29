#ifndef FEATUREDEM_H
#define FEATUREDEM_H

#include <include/common.h>
#include <iostream>
#include <utility>

/* Feature Detector--------Region Based FAST Detector
 *         Extractor-------ORB descriptor
 *         Match Checker---Check by ORB distance
// */



/*
 *  Region Based FAST Detector
 *  Regions:
 *  | 0 | 1 | 2 | 3 |
 *  | 4 | 5 | 6 | 7 |
 *  | 8 | 9 | 10| 11|
 *  | 12| 13| 14| 15|
 *  //Detect FAST features
 *  //Devided all features into 16 regions
 *  //For every region, select features by Harris index and boundary size
// */

using namespace std;


class FeatureDEM
{
public:


  FeatureDEM(const int image_width,
             const int image_height,
             const Vec6 f_para);
  ~FeatureDEM();

  void detect(const cv::Mat& img,
              vector<cv::Point2f>& newPts);

//  void detect_conventional(const cv::Mat& img,
//              vector<Vec2>& pts,
//              vector<cv::Mat>& descriptors);

  void redetect(const cv::Mat& img,
                const vector<Vec2>& existedPts,
                vector<cv::Point2f>& newPts,
                int &newKeyPtscount);

private:
  int width;
  int height;
  int regionWidth;
  int regionHeight;
  int boundary_dis;
  unsigned int max_region_feature_num;
  unsigned int min_region_feature_num;
  int gftt_num;
  double gftt_ql;
  int gftt_dis;
  vector<pair<cv::Point2f,float>> regionKeyPts[16];
  cv::Mat detectorMask[16];

  void calHarrisR(const cv::Mat& img, cv::Point2f& Pt, float &R);

  void fillIntoRegion(const cv::Mat& img,
                      const vector<cv::Point2f>& pts,
                      vector<pair<cv::Point2f,float>> (&region)[16],
                      bool  existed_features);




};//class RegionFeatureDetector

#endif // FEATUREDEM_H

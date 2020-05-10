#ifndef FEATUREDEM_H
#define FEATUREDEM_H

#include <include/common.h>
#include <iostream>
#include <utility>

/* Feature Detector--------Region Based FAST Detector
 *         Extractor-------ORB descriptor
 *         Match Checker---Check by ORB distance
 *  * /



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

#define MAX_REGION_FREATURES_NUM (30)
#define MIN_REGION_FREATURES_NUM (20)


#define BOUNDARYBOXSIZE          (5)


using namespace std;


class FeatureDEM
{
public:


  FeatureDEM(const int image_width,
             const int image_height,
             int boundaryBoxSize=BOUNDARYBOXSIZE);
  ~FeatureDEM();

  void detect(const cv::Mat& img,
              vector<Vec2>& newPts);

//  void detect_conventional(const cv::Mat& img,
//              vector<Vec2>& pts,
//              vector<cv::Mat>& descriptors);

  void redetect(const cv::Mat& img,
                const vector<Vec2>& existedPts,
                vector<Vec2>& newPts,
                int &newKeyPtscount);

private:
  int width;
  int height;
  int regionWidth;
  int regionHeight;
  int boundary_dis;
  vector<pair<cv::Point2f,float>> regionKeyPts[16];
  cv::Mat detectorMask[16];

  void calHarrisR(const cv::Mat& img, cv::Point2f& Pt, float &R);

  void fillIntoRegion(const cv::Mat& img,
                      const vector<cv::Point2f>& pts,
                      vector<pair<cv::Point2f,float>> (&region)[16],
                      bool  existed_features);




};//class RegionFeatureDetector

#endif // FEATUREDEM_H

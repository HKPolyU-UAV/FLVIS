#ifndef COMMON_DEF_H
#define COMMON_DEF_H

#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>

#include "../3rdPartLib/Sophus/sophus/so3.h"
#include "../3rdPartLib/Sophus/sophus/se3.h"

//#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/se3quat.h>

using namespace Eigen;
using namespace std;

using namespace Sophus;

typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<int, 3, 1> Vec3I;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 9, 1> Vec9;
typedef Eigen::Matrix<double, 12, 1> Vec12;
typedef Eigen::Matrix<double, 15, 1> Vec15;
typedef Eigen::Matrix<double, 16, 1> Vec16;
typedef Eigen::Matrix<double, 1, 1> Mat1x1;
typedef Eigen::Matrix<double, 1, 4> Mat1x4;
typedef Eigen::Matrix<double, 3, 3> Mat3x3;
typedef Eigen::Matrix<double, 3, 4> Mat3x4;
typedef Eigen::Matrix<double, 4, 4> Mat4x4;
typedef Eigen::Matrix<double, 6, 6> Mat6x6;
typedef Eigen::Matrix<double, 9, 9> Mat9x9;
typedef Eigen::Matrix<double, 12, 12> Mat12x12;
typedef Eigen::Matrix<double, 15, 15> Mat15x15;
typedef Eigen::Matrix<double, 15, 6> Mat15x6;
typedef Eigen::Matrix<double, 6, 15> Mat6x15;
typedef Eigen::Matrix<double, 9, 15> Mat9x15;
typedef Eigen::Matrix<double, 15, 12> Mat15x12;
typedef Eigen::Matrix<double, 15, 9> Mat15x9;
typedef Eigen::Matrix<double, 3, 15> Mat3x15;
typedef Eigen::Matrix<double, 15, 3> Mat15x3;
typedef Eigen::Matrix<double, 1, 15> Mat1x15;
typedef Eigen::Matrix<double, 15, 1> Mat15x1;



typedef pcl::PointXYZ PointP;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZI PointI;

typedef pcl::PointCloud<PointP> PointCloudP;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointI> PointCloudI;



//transfor descriptors to vector of Mat(cv)
inline void descriptors_to_vMat(const cv::Mat& descriptorMat, vector<cv::Mat>& vecDescriptorMat)
{
    vecDescriptorMat.clear();
    for(int i=0; i<descriptorMat.size().height;i++)
    {
        vecDescriptorMat.push_back(descriptorMat.row(i));
    }
}
//transfer vector of Mat to descriptors
inline void vMat_to_descriptors(const cv::Mat& descriptorMat, vector<cv::Mat>& vecDescriptorMat)
{
  for(uint64_t i=0; i<vecDescriptorMat.size();i++)
  {
      descriptorMat.row(i) = vecDescriptorMat[i] + 0;// if delete 0, descriptorMat.row(i) will be zeros.
  }
}

//transfor 2d point to cvP2f
inline cv::Point2f Vec2_to_cvP2f(const Vec2 pt)
{
    return cv::Point2f(pt[0],pt[1]);
}

//transfor 3d point to cvP3f
inline cv::Point3f Vec3_to_cvP3f(const Vec3 pt)
{
    return cv::Point3f(pt[0],pt[1],pt[2]);
}

//transfor (vector of 2d point) to (vector of cvP2f)
inline vector<cv::Point2f> vVec2_2_vcvP2f(const vector<Vec2>& pt2ds)
{
    vector<cv::Point2f> ret;
    ret.clear();
    for(size_t i=0; i<pt2ds.size(); i++)
        ret.push_back(cv::Point2f(pt2ds.at(i)[0],pt2ds.at(i)[1]));
    return ret;
}

inline cv::Mat Mat3x3_to_cvMat(const Mat3x3 R)
{
    cv::Mat ret;
    double data[9] = { R(0,0), R(0,1), R(0,2),
                       R(1,0), R(1,1), R(1,2),
                       R(2,0), R(2,1), R(2,2)};
    ret = cv::Mat(3, 3, CV_64F, data);
    return ret;
}
//transfor Vec3 to cv tvec
inline cv::Mat Vec3_to_cvMat(const Vec3 t)
{
    cv::Mat ret = cv::Mat::zeros(3, 1, CV_64FC1);;

    ret.at<double>(0,0) = t(0,0);
    ret.at<double>(1,0) = t(1,0);
    ret.at<double>(2,0) = t(2,0);

    return ret;
}
//transfor cv rotation to Mat3x3
inline Mat3x3 cvMat_to_Mat3x3(const cv::Mat R)
{
    Mat3x3 ret;
    ret<<R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
            R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
            R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);
    return ret;
}

//transfor cv translation to Vec3
inline Vec3 cvMat_to_Vec3(const cv::Mat t)
{
    return Vec3(t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0));
}

inline SE3 SE3_from_rvec_tvec(cv::Mat rvec, cv::Mat tvec)
{
    cv::Mat R_;
    cv::Rodrigues ( rvec, R_ );
    Mat3x3 R=cvMat_to_Mat3x3(R_);
    Vec3   t=cvMat_to_Vec3(tvec);
    return SE3(R,t);
}

inline void SE3_to_rvec_tvec(const SE3 pose, cv::Mat &rvec, cv::Mat &tvec)
{
    Quaterniond q = pose.unit_quaternion();
    cv::Mat t_,r_;
    tvec = Vec3_to_cvMat(pose.translation());
    cv::Mat R_ = Mat3x3_to_cvMat(q.toRotationMatrix());
    cv::Rodrigues ( R_, rvec );
}

inline SE3 SE3_from_g2o(g2o::SE3Quat &g2o_pose)
{
  Eigen::Matrix<double,4,4> eigMat = g2o_pose.to_homogeneous_matrix();
  return SE3(eigMat.block(0,0,3,3),eigMat.col(3).head(3));

}
inline g2o::SE3Quat SE3_to_g2o(SE3 &se3_pose)
{
  Quaterniond q = se3_pose.unit_quaternion();
  Vector3d t = se3_pose.translation();

  return g2o::SE3Quat(q, t);

}


#endif // COMMON_H

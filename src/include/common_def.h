#ifndef COMMON_H
#define COMMON_H

#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include "../3rdPartLib/Sophus/sophus/so3.h"
#include "../3rdPartLib/Sophus/sophus/se3.h"

using namespace Eigen;
using namespace std;
using namespace cv;
using namespace Sophus;

typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 9, 1> Vec9;
typedef Eigen::Matrix<double, 12, 1> Vec12;
typedef Eigen::Matrix<double, 15, 1> Vec15;
typedef Eigen::Matrix<double, 16, 1> Vec16;

typedef Eigen::Matrix<double, 1, 1> Mat1x1;
typedef Eigen::Matrix<double, 3, 3> Mat3x3;
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


#endif // COMMON_H

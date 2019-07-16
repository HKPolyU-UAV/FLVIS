#ifndef QUATERNION_MULTIPLICATION_H
#define QUATERNION_MULTIPLICATION_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

/*
 *  @brief Create a skew-symmetric matrix from a 3-element vector.
 *  @note Performs the operation:
 *  w   ->  [  0 -w3  w2]
 *          [ w3   0 -w1]
 *          [-w2  w1   0]
 */
inline Matrix3f skew_symmetric_from_vector(const Vector3f v) {
  Matrix3f skew;
  skew(0, 0) = 0;     skew(0, 1) = -v(2);  skew(0, 2) = v(1);
  skew(1, 0) = v(2);  skew(1, 1) = 0;      skew(1, 2) = -v(0);
  skew(2, 0) = -v(1); skew(2, 1) = v(0);   skew(2, 2) = 0;
  return skew;
}

inline Quaternionf q_multi_q(const Quaternionf a, const Quaternionf b)
{
  Quaternionf q;
  q.w() = a.w()*b.w() - a.x()*b.x() - a.y()*b.y() - a.z()*b.z();
  q.x() = a.x()*b.w() + a.w()*b.x() - a.z()*b.y() + a.y()*b.z();
  q.y() = a.y()*b.w() + a.z()*b.x() + a.w()*b.y() - a.x()*b.z();
  q.z() = a.z()*b.w() - a.y()*b.x() + a.x()*b.y() + a.w()*b.z();
  return q;
}

inline Quaternionf scalar_multi_q(const float a, const Quaternionf b)
{
  Quaternionf q;
  q.w() = a*b.w();
  q.x() = a*b.x();
  q.y() = a*b.y();
  q.z() = a*b.z();
  return q;
}

inline Quaternionf q_plus_q(const Quaternionf a, const Quaternionf b)
{
  Quaternionf q;
  q.w() = a.w()+b.w();
  q.x() = a.x()+b.x();
  q.y() = a.y()+b.y();
  q.z() = a.z()+b.z();
  return q;
}
#endif // QUATERNION_MULTIPLICATION_H

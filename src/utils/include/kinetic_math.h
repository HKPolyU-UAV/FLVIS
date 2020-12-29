#ifndef KINETIC_MATH
#define KINETIC_MATH

#include <include/common.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//In ENU frame: X-roll Y-pitch Z-yaw
//Euler Angle follow Yaw–Pitch–Roll Z1*Y2*X3
//Hence: R=R_yaw*R_pitch*R_roll
//       R=Rz*Ry*Rx


using namespace Eigen;

inline Matrix3d rpy2R(const Vec3 rpy)
{
    double r=rpy(0);
    double p=rpy(1);
    double y=rpy(2);

    double cy = cos(y);
    double sy = sin(y);
    double cp = cos(p);
    double sp = sin(p);
    double cr = cos(r);
    double sr = sin(r);

    double r11 = cy*cp;
    double r12 = cy*sp*sr-sy*cr;
    double r13 = cy*sp*cr+sy*sr;

    double r21 = sy*cp;
    double r22 = sy*sp*sr+cy*cr;
    double r23 = sy*sp*cr-cy*sr;

    double r31 = -sp;
    double r32 = cp*sr;
    double r33 = cp*cr;

    Matrix3d R;

    R << r11 , r12 , r13 , r21 , r22 , r23 , r31 , r32 , r33;

    return R;
}

inline Vec3 R2rpy(const Matrix3d R)
{
    Vec3 rpy;
    double roll, pitch, yaw;
//    double r12 = R(0,1);
//    double r22 = R(1,1);
//    double r31 = R(2,0);
//    double r32 = R(2,1);
//    double r33 = R(2,2);

    roll  = atan2(R(2,1), R(2,2));
    //pitch = asin(-R(2,0))
    pitch = atan2(-R(2,0), sqrt(R(2,1) * R(2,1) + R(2,2) * R(2,2)));
    yaw   = atan2(R(1,0), R(0,0));

    rpy(0) = roll;
    rpy(1) = pitch;
    rpy(2) = yaw;

    return rpy;
}

inline Quaterniond R2Q(const Matrix3d R)
{
    Quaterniond q;
    q = Quaterniond(R);
    return q;

}

inline Matrix3d Q2R(const Quaterniond q)
{
    return q.toRotationMatrix();
}

inline Quaterniond rpy2Q(const Vec3 rpy)
{
    return R2Q(rpy2R(rpy));
}

inline Vec3 Q2rpy(const Quaterniond q)
{
    return R2rpy(Q2R(q));
}
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
//q1 multi q2 = A(q2)*q1
//A=matrix[
//w2 −x2 −y2 −z2
//x2  w2  z2 −y2
//y2 −z2  w2  x2
//z2  y2 -x2  w2]
inline Quaterniond q1_multi_q2(const Quaterniond q1, const Quaterniond q2)
{
  Quaterniond q;
  q.w() = q2.w()*q1.w() - q2.x()*q1.x() - q2.y()*q1.y() - q2.z()*q1.z();
  q.x() = q2.x()*q1.w() + q2.w()*q1.x() + q2.z()*q1.y() - q2.y()*q1.z();
  q.y() = q2.y()*q1.w() - q2.z()*q1.x() + q2.w()*q1.y() + q2.x()*q1.z();
  q.z() = q2.z()*q1.w() + q2.y()*q1.x() - q2.x()*q1.y() + q2.w()*q1.z();
  return q;
}

inline Quaterniond scalar_multi_q(const float a, const Quaterniond b)
{
  Quaterniond q;
  q.w() = a*b.w();
  q.x() = a*b.x();
  q.y() = a*b.y();
  q.z() = a*b.z();
  return q;
}

inline Quaterniond q_plus_q(const Quaterniond a, const Quaterniond b)
{
  Quaterniond q;
  q.w() = a.w()+b.w();
  q.x() = a.x()+b.x();
  q.y() = a.y()+b.y();
  q.z() = a.z()+b.z();
  return q;
}
#endif // QUATERNION_MULTIPLICATION_H

#include "include/triangulation.h"
#include <Eigen/Dense>

Triangulation::Triangulation()
{

}

Vec3 Triangulation::triangulationPt(Vec2 pt1, Vec2 pt2, Mat3x4 projection_matrix1, Mat3x4 projection_matrix2)
{
    double u1=pt1(0,0);
    double v1=pt1(1,0);
    double u2=pt2(0,0);
    double v2=pt2(1,0);

    Mat1x4 PT1_pm1=projection_matrix1.row(0);
    Mat1x4 PT2_pm1=projection_matrix1.row(1);
    Mat1x4 PT3_pm1=projection_matrix1.row(2);

    Mat1x4 PT1_pm2=projection_matrix2.row(0);
    Mat1x4 PT2_pm2=projection_matrix2.row(1);
    Mat1x4 PT3_pm2=projection_matrix2.row(2);

    Mat4x4 A;

    A.row(0)=v1*PT3_pm1-PT2_pm1;
    A.row(1)=PT1_pm1-u1*PT3_pm1;
    A.row(2)=v2*PT3_pm2-PT2_pm2;
    A.row(3)=PT1_pm2-u2*PT3_pm2;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
    Mat4x4 V = svd.matrixV();

    V(0, 3) /= V(3, 3);
    V(1, 3) /= V(3, 3);
    V(2, 3) /= V(3, 3);

    return V.block<3, 1>(0, 3);
}

bool Triangulation::trignaulationPtFromStereo(Vec2 pt0, Vec2 pt1,
                                              Mat3x3 c0Matrix, Mat3x3 c1Matrix,
                                              SE3 T_c1_c0,
                                              Vec3 &pt3d_c)
{
    SE3 T_c1_w = T_c1_c0;
    Mat3x4 T0,T1;
    T0.topLeftCorner(3,3).setIdentity();
    T0.topRightCorner(3,1).setZero();
    T1.topLeftCorner(3,3)=T_c1_w.rotation_matrix();
    T1.topRightCorner(3,1)=T_c1_w.translation();
    Mat3x4 P0,P1;//Projection Matrix
    P0 = c0Matrix*T0;
    P1 = c1Matrix*T1;
    pt3d_c = triangulationPt(pt0,pt1,P0,P1);
    if(pt3d_c[2]<0 || pt3d_c[2]>8)
    {
        return false;
    }else
    {
        return true;
    }
}

Vec3 Triangulation::triangulationPt(Vec2 pt1, Vec2 pt2,
                                    SE3 T_c_w1, SE3 T_c_w2,
                                    double fx, double fy, double cx, double cy)
{
    Mat3x3 K;//Kamera matrix
    K<<fx,0,cx,0,fy,cy,0,0,1;

    Mat3x4 T1,T2;//Transformation matrix
    T1.topLeftCorner(3,3)=T_c_w1.rotation_matrix();
    T1.topRightCorner(3,1)=T_c_w1.translation();
    T2.topLeftCorner(3,3)=T_c_w2.rotation_matrix();
    T2.topRightCorner(3,1)=T_c_w2.translation();

    Mat3x4 P1,P2;//Projection Matrix
    P1 = K*T1;
    P2 = K*T2;
    return triangulationPt(pt1,pt2,P1,P2);
}

Vec2 Triangulation::reProjection(Vec3 pt, SE3 T_c_w, double fx, double fy, double cx, double cy)
{
    Mat3x3 K;//Kamera matrix
    K<<fx,0,cx,0,fy,cy,0,0,1;

    Mat3x4 T;//Transformation matrix
    T.topLeftCorner(3,3)=T_c_w.rotation_matrix();
    T.topRightCorner(3,1)=T_c_w.translation();


    Mat3x4 P=K*T;//Projection Matrix
    //    cout << K << endl;
    //    cout << T << endl;
    //    cout << P << endl;

    Vec4 pt3d_homogeneous = Vec4(pt(0),pt(1),pt(2),1);
    Vec3 pt2d_homogeneous=P*pt3d_homogeneous;
    //    cout << pt3d_homogeneous.transpose() << endl;
    //    cout << pt2d_homogeneous.transpose() << endl;
    return Vec2(pt2d_homogeneous(0)/pt2d_homogeneous(2),pt2d_homogeneous(1)/pt2d_homogeneous(2));
}

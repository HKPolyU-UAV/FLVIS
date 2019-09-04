#include "include/triangulation.h"
#include <iostream>
#include <iterator>
#include <random>
#include <chrono>
#include <ctime>
//intrinsics: [239.0838, 238.686676, 239.0838, 134.681549]#fx fy cx cy

#define BASELINE_DIST (0.15)
#define TARGER_DISTANCE (8.0)
#define GAUSSIAN_SIGMA (0.05)
#define TEST_NUM (10000)

int main( int argc, char** argv )
{
    cout << "test triangulation function" << endl;
    double fx=239.0838;
    double fy=238.686676;
    double cx=239.0838;
    double cy=134.681549;

    Mat3x3 R;
    R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Vec3   t1=Vec3(0,0,0);
    Vec3   t2=Vec3(BASELINE_DIST,0,0);// ||Base Line||=0.05
    SE3    T_c_w1=SE3(R,t1).inverse();
    SE3    T_c_w2=SE3(R,t2).inverse();
    Vec3 pt_orig=Vec3(0.0, 0.0, TARGER_DISTANCE);
    Vec2 pt_in_1 = Triangulation::reProjection(pt_orig,T_c_w1,fx,fy,cx,cy);
    Vec2 pt_in_2 = Triangulation::reProjection(pt_orig,T_c_w2,fx,fy,cx,cy);
    cout << "reproject point (" << pt_orig.transpose() << ") to the camera1 at (" << pt_in_1.transpose() <<")"<< endl;
    cout << "reproject point (" << pt_orig.transpose() << ") to the camera2 at (" << pt_in_2.transpose() <<")"<< endl;
    Vec3 t_rst = Triangulation::triangulationPt(pt_in_1,pt_in_2,
                                                T_c_w1,T_c_w2,
                                              fx,fy,cx,cy);
    cout << "triangulation with out noise(" << t_rst.transpose() << ")"<< endl;

    const double mean = 0.0;
    const double stddev = GAUSSIAN_SIGMA;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    vector<Vec2> pts_in_1;
    vector<Vec2> pts_in_2;
    vector<Vec3> pts_result;
    pts_in_1.clear();
    pts_in_2.clear();
    for(int i=0; i<TEST_NUM; i++)
    {
        Vec2 pt1_mit_noise = Vec2(pt_in_1(0)+dist(generator),pt_in_1(1)+dist(generator));
        Vec2 pt2_mit_noise = Vec2(pt_in_2(0)+dist(generator),pt_in_2(1)+dist(generator));
        pts_in_1.push_back(pt1_mit_noise);
        pts_in_2.push_back(pt2_mit_noise);
    }
    cout << "Triangulate " << TEST_NUM << " pts cost: ";
    auto start = std::chrono::system_clock::now();
    for(int i=0; i<TEST_NUM; i++)
    {
        pts_result.push_back(Triangulation::triangulationPt(pts_in_1.at(i),pts_in_2.at(i),
                                                            T_c_w1,T_c_w2,
                                                          fx,fy,cx,cy));
    }
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    cout << elapsed_seconds.count()*1000 << "ms" << endl;

    for(int i=0; i<TEST_NUM; i+=(int)(TEST_NUM/50))
    {
        cout << pts_result.at(i).transpose() << endl;
    }
}

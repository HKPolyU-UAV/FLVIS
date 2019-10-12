#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <deque>

#include <include/yamlRead.h>
#include <include/cv_draw.h>

#include <include/keyframe_msg.h>
#include <vo_nodelet/KeyFrame.h>
#include <geometry_msgs/Vector3.h>


#include "g2o/config.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/icp/types_icp.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"

#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


using namespace cv;
using namespace std;

namespace vo_nodelet_ns
{



std::deque<KeyFrameStruct> kfs;

class VOLocalMapNodeletClass : public nodelet::Nodelet
{
public:
    VOLocalMapNodeletClass()  {;}
    ~VOLocalMapNodeletClass() {;}

private:
    ros::Subscriber sub_kf;
    int image_width,image_height;
    Mat cameraMatrix,distCoeffs;
    Mat diplay_img;
    double fx,fy,cx,cy;
    bool optimizer_initialized;
    g2o::SparseOptimizer optimizer;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    g2o::OptimizationAlgorithmLevenberg* solver;
    g2o::CameraParameters* cam_params;

    vector<uint64_t> optimizer_lm_id;

    void frame_callback(const vo_nodelet::KeyFrameConstPtr& msg)
    {
        KeyFrameStruct kf;

        KeyFrameMsg::unpack(msg,kf.frame_id,kf.img,kf.lm_id,kf.lm_2d,kf.lm_3d,kf.lm_descriptor,kf.T_c_w);

        kfs.push_back(kf);


        if(kfs.size()>7)//fix window BA
        {
            if(optimizer_initialized)
            {
                //remove outside of window vertex (poses & landmarks) and edges


                //add new vertex and edges

            }
            else//initialize the optimizer
            {
                for(int f_idx = 0; f_idx<7; f_idx++)//add pose
                {
                    for(int lm_idx=0; lm_idx < 10; lm_idx++)//add landmarks
                    {
//                        g2o* point = new g2o::pointType(); // 伪码
//                        point->setId(index++);
//                        point->setEstimate(/*something*/);
//                        // point->setMarginalized(true); // 该点在解方程时进行Schur消元

//                        optimizer->addVertex(point);
                    }
                }

                optimizer.setVerbose(false);
                solver = new g2o::OptimizationAlgorithmLevenberg(
                            g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));
                optimizer.setAlgorithm(solver);
                cam_params = new g2o::CameraParameters (((fx+fy)/2), Vec2(cx,cy), 0.);
                cam_params->setId(0);

            }
            //visualization
            for(size_t i=0; i<8; i++)
            {
                Mat dst;
                pyrDown( kfs.at(i).img, dst, Size( image_width/2, image_height/2));
                cvtColor(dst,dst,CV_GRAY2BGR);
                vector<Vec2> lm_2d_half;
                for(size_t j=0; j<kfs.at(i).lm_2d.size(); j++)
                {
                    Vec2 p2d = kfs.at(i).lm_2d.at(j);
                    Vec2 p2d_half = 0.5*p2d;
                    lm_2d_half.push_back(p2d_half);
                }
                drawKeyPts(dst,vVec2_2_vcvP2f(lm_2d_half));
                int start_x = i%4*(image_width/2)+(i%4);
                int start_y = i/4*(image_height/2);
                Mat diplay_img_roi(diplay_img, Rect(start_x, start_y, dst.cols, dst.rows));
                dst.copyTo(diplay_img_roi);

            }
            //imshow("dispaly_img", diplay_img);
            //waitKey(1);

            kfs.pop_front();
            //DISPLAY 8 IMAGES
        }

        cout << endl << "Local Map BA Callback" << endl;
    }

    virtual void onInit()
    {
        ros::NodeHandle nh = getNodeHandle();

        string configFilePath;
        nh.getParam("/yamlconfigfile",   configFilePath);
        image_width  = getIntVariableFromYaml(configFilePath,"image_width");
        image_height = getIntVariableFromYaml(configFilePath,"image_height");
        cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath);
        distCoeffs = distCoeffsFromYaml(configFilePath);
        fx = cameraMatrix.at<double>(0,0);
        fy = cameraMatrix.at<double>(1,1);
        cx = cameraMatrix.at<double>(0,2);
        cy = cameraMatrix.at<double>(1,2);
        cout << "cameraMatrix:" << endl << cameraMatrix << endl
             << "distCoeffs:" << endl << distCoeffs << endl
             << "image_width: "  << image_width << " image_height: "  << image_height << endl
             << "fx: "  << fx << " fy: "  << fy <<  " cx: "  << cx <<  " cy: "  << cy << endl;
        diplay_img.create(image_height+1,image_width*2+5,CV_8UC(3));
        optimizer_initialized = false;

        sub_kf = nh.subscribe<vo_nodelet::KeyFrame>(
                    "/vo_kf",
                    10,
                    boost::bind(&VOLocalMapNodeletClass::frame_callback, this, _1));

    }




};//class VOLocalMapNodeletClass
}//namespace vo_nodelet_ns



PLUGINLIB_EXPORT_CLASS(vo_nodelet_ns::VOLocalMapNodeletClass, nodelet::Nodelet)


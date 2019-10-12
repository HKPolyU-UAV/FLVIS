#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <deque>

#include <include/yamlRead.h>

#include <include/keyframe_msg.h>
#include <vo_nodelet/KeyFrame.h>
#include <geometry_msgs/Vector3.h>

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

struct KeyFrameStruct {
    cv::Mat img;
    vector<uint64_t> lm_id;
    vector<Vec2> lm_2d;
    vector<Vec3> lm_3d;
    vector<Mat> lm_descriptors;
    SE3 T_c_w;
};

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
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    g2o::BlockSolver_6_3* solver_ptr;
    g2o::OptimizationAlgorithmLevenberg* solver;
    vector<uint64_t> optimizer_lm_id;

    void frame_callback(const vo_nodelet::KeyFrameConstPtr& msg)
    {
        KeyFrameStruct kf;
        KeyFrameMsg::unpack(msg,kf.img,kf.lm_id,kf.lm_2d,kf.lm_3d,kf.lm_descriptors,kf.T_c_w);
        kfs.push_back(kf);


        if(kfs.size()>7)//fix window BA
        {
            if(optimizer_initialized)
            {
                //add new land marks
                //

            }
            else//initialize the optimizer
            {//get all pose and landmarks
//                linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
//                solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
//                solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//                optimizer.setAlgorithm(solver);

            }
            for(int i=0; i<8; i++)
            {


                Mat dst;
                pyrDown( kfs.at(i).img, dst, Size( image_width/2, image_height/2));
                cout << "here" << i <<endl;
                cvtColor(dst,dst,CV_GRAY2BGR);
                int start_x = i%4*(image_width/2)+(i%4);
                int start_y = i/4*(image_height/2);
                cout << start_x << "," << start_y << "|" << dst.cols <<"," << dst.rows << endl;
                Mat diplay_img_roi(diplay_img, Rect(start_x, start_y, dst.cols, dst.rows));
                cout << "here" << i <<endl;
                dst.copyTo(diplay_img_roi);
                cout << "here" << i <<endl;
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


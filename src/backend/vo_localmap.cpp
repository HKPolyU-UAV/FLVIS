#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <deque>

#include <include/yamlRead.h>
#include <include/correction_inf_msg.h>
#include <include/keyframe_msg.h>
#include <flvis/KeyFrame.h>
#include <include/depth_camera.h>
#include <geometry_msgs/Vector3.h>
#include <include/poselmbag.h>

#include "g2o/config.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/icp/types_icp.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"


#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


using namespace cv;
using namespace std;



static int64_t edge_id;


namespace flvis_ns
{

std::deque<KeyFrameStruct> kfs;
PoseLMBag* bag;

enum LMOPTIMIZER_STATE{
    UN_INITIALIZED,
    SLIDING_WINDOW,
    OPTIMIZING,
    FAIL};

class LocalMapNodeletClass : public nodelet::Nodelet
{
public:
    LocalMapNodeletClass()  {;}
    ~LocalMapNodeletClass() {;}

private:
    ros::Subscriber sub_kf;
    CorrectionInfMsg* pub_correction_inf;

    enum TYPEOFCAMERA cam_type;
    double fx,fy,cx,cy;
    int fix_window_optimizer_size;

    g2o::CameraParameters* cam_params;
    LMOPTIMIZER_STATE optimizer_state;
    g2o::SparseOptimizer optimizer;
    vector<g2o::EdgeProjectXYZ2UV*> edges;


    void frame_callback(const flvis::KeyFrameConstPtr& msg)
    {
        if(msg->command==KFMSG_CMD_RESET_LM)
        {
            optimizer_state=UN_INITIALIZED;
            bag->reset();
            kfs.clear();
            optimizer.clear();
            edges.clear();
            cout << "reset the local map" << endl;
            return;
        }
        KeyFrameStruct kf;
        ros::Time tt;
        KeyFrameMsg::unpack(msg,
                            kf.frame_id,
                            kf.img,
                            kf.d_img,
                            kf.lm_count,
                            kf.lm_id,
                            kf.lm_2d,
                            kf.lm_3d,
                            kf.lm_descriptor,
                            kf.T_c_w,
                            tt);


        kfs.push_back(kf);
        //        cout << "LocalMap: inframe_callback function" << endl;
        //        cout << "LocalMap: optimizer_state is: " << optimizer_state << endl;

        switch(optimizer_state)
        {
        case OPTIMIZING:
            break;
        case UN_INITIALIZED:
            if(1){
                cout << "LocalMap: optimizer uninitialized" << endl;
                if(kfs.size()>=fix_window_optimizer_size)
                {
                    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>());
                    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));
                    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
                    optimizer.setAlgorithm (solver);
                    for(int f_idx = 0; f_idx<fix_window_optimizer_size; f_idx++)//add pose
                    {
                        bag->addPose(kfs.at(f_idx).frame_id,
                                     kfs.at(f_idx).T_c_w);
                        //cout << "lm_cout " << kfs.at(f_idx).lm_count << " " << kfs.at(f_idx).lm_3d.size() << endl;
                        for(int lm_idx=0; lm_idx < kfs.at(f_idx).lm_count; lm_idx++)//add landmarks
                        {
                            bag->addLMObservation(kfs.at(f_idx).lm_id.at(lm_idx),
                                                  kfs.at(f_idx).lm_3d.at(lm_idx));
                        }
                    }
                    cout << "LocalMap: Initialize Optimizer*****" << endl;
                    //STEP1: Add Camera Pose Vertex
                    //STEP2: Add LandMark Vertex
                    //STEP3: Add All Observation Edge

                    //STEP1:
                    vector<POSE_ITEM> poses;
                    int oldest_idx1 = bag->getOldestPoseInOptimizerIdx();
                    int oldest_idx2 = (oldest_idx1+1);
                    if(oldest_idx2 == fix_window_optimizer_size) oldest_idx2 = 0;
                    bag->getAllPoses(poses);
                    for(std::vector<POSE_ITEM>::iterator it = poses.begin(); it != poses.end(); ++it)
                    {
                        g2o::VertexSE3Expmap* v_pose = new g2o::VertexSE3Expmap();
                        v_pose->setId(it->pose_id);//fix first two items
                        //if ((it->pose_id==oldest_idx1) || (it->pose_id==oldest_idx2))
                        if (it->pose_id==oldest_idx1)
                        {
                            v_pose->setFixed(true);
                        }
                        v_pose->setEstimate(g2o::SE3Quat(it->pose.so3().unit_quaternion().toRotationMatrix(),
                                                         it->pose.translation()));
                        optimizer.addVertex(v_pose);
                    }

                    //STEP2: Add LandMark Vertex;
                    vector<LM_ITEM> lms;
                    bag->getAllLMs(lms);
                    for(std::vector<LM_ITEM>::iterator it = lms.begin(); it != lms.end(); ++it) {
                        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
                        point->setId (it->id);
                        point->setEstimate (it->p3d_w);
                        point->setMarginalized ( true );
                        optimizer.addVertex (point);
                    }

                    //STEP3: Add All Observation Edge;
                    g2o::CameraParameters* camera = new g2o::CameraParameters(((fx+fy)/2.0), Eigen::Vector2d(cx, cy), 0 );
                    camera->setId(0);
                    optimizer.addParameter(camera);
                    edge_id = 0;
                    edges.clear();
                    for(int f_idx = 0; f_idx<fix_window_optimizer_size; f_idx++)//add pose
                    {
                        int64_t pose_vertex_idx = bag->getPoseIdByReleventFrameId(kfs.at(f_idx).frame_id);
                        //cout << pose_vertex_idx << endl;
                        for(int lm_idx=0; lm_idx < kfs.at(f_idx).lm_count; lm_idx++)//add landmarks
                        {
                            g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
                            edge->setId(edge_id);
                            edge_id++;
                            int64_t lm_vertex_idx = kfs.at(f_idx).lm_id.at(lm_idx);
                            edge->setVertex(0,dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(  lm_vertex_idx)));
                            edge->setVertex(1,dynamic_cast<g2o::VertexSE3Expmap*>  (optimizer.vertex(pose_vertex_idx)));
                            edge->setMeasurement(kfs.at(f_idx).lm_2d.at(lm_idx));
                            edge->setInformation(Eigen::Matrix2d::Identity() );
                            edge->setParameterId(0,0);
                            edge->setRobustKernel(new g2o::RobustKernelHuber());
                            optimizer.addEdge(edge);
                            edges.push_back(edge);
                        }
                    }
                    optimizer_state = OPTIMIZING;
                }
                else//if(kfs.size()>=FIX_WINDOW_OPTIMIZER_SIZE)
                {
                    return;
                }
            }
            break;//switch(optimizer_state) case UN_INITIALIZED:

        case SLIDING_WINDOW:
            if(1)
            {

                //STEP1: Delete Oldest Frame and idle LM;
                //STEP2: Add new Frame, LM and Observation;

                optimizer.removeVertex(dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(bag->getOldestPoseInOptimizerIdx())));
                for(auto id:kfs.at(0).lm_id)
                {
                    if(bag->removeLMObservation(id))
                    {
                        optimizer.removeVertex(dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(id)));
                    }
                }
                //STEP2:
                bag->addPose(kfs.back().frame_id,
                             kfs.back().T_c_w);
                g2o::VertexSE3Expmap* v_pose = new g2o::VertexSE3Expmap();
                v_pose->setId(bag->getNewestPoseInOptimizerIdx());
                v_pose->setEstimate(g2o::SE3Quat(kfs.back().T_c_w.so3().unit_quaternion().toRotationMatrix(),
                                                 kfs.back().T_c_w.translation()));
                optimizer.addVertex(v_pose);
                optimizer.vertex(bag->getOldestPoseInOptimizerIdx())->setFixed(true);
                for(int i=0; i < kfs.back().lm_count; i++)
                {
                    if(bag->addLMObservationSlidingWindow(kfs.back().lm_id.at(i),
                                                          kfs.back().lm_3d.at(i)))
                    {
                        g2o::VertexSBAPointXYZ* v_lm = new g2o::VertexSBAPointXYZ();
                        v_lm->setId (kfs.back().lm_id.at(i));
                        v_lm->setEstimate (kfs.back().lm_3d.at(i));
                        v_lm->setMarginalized ( true );
                        optimizer.addVertex (v_lm);
                    }
                }
                g2o::HyperGraph::EdgeSet es = optimizer.edges();
                vector<g2o::HyperGraph::Edge*> v(es.begin(), es.end());
                edges.clear();
                for (auto v_itm:v)
                {
                    edges.push_back(dynamic_cast<g2o::EdgeProjectXYZ2UV*>(v_itm));
                }
                for(int i=0; i < kfs.back().lm_count; i++)//add landmarks
                {
                    g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
                    edge->setId(edge_id);
                    edge_id++;
                    int64_t lm_vertex_idx = kfs.back().lm_id.at(i);
                    edge->setVertex(0,dynamic_cast<g2o::VertexSBAPointXYZ*>
                                    (optimizer.vertex(  lm_vertex_idx)));
                    edge->setVertex(1,dynamic_cast<g2o::VertexSE3Expmap*>
                                    (optimizer.vertex(bag->getNewestPoseInOptimizerIdx())));
                    edge->setMeasurement(kfs.back().lm_2d.at(i));
                    edge->setInformation(Eigen::Matrix2d::Identity() );
                    edge->setParameterId(0,0);
                    edge->setRobustKernel(new g2o::RobustKernelHuber());
                    optimizer.addEdge(edge);
                    edges.push_back(edge);
                }
                optimizer_state=OPTIMIZING;
            }
            break;//switch(optimizer_state) case SLIDING_WINDOW:
        case FAIL:
            cout << "LocalMap:  --------------------" << endl;
            break;//switch(optimizer_state) case FAIL:
        default:
            cout << "LocalMap:  Default?? sth wrong" << endl;

        }
        if(optimizer_state==OPTIMIZING)
        {
            //cout << "LocalMap: optimizing" << endl;
            CorrectionInfStruct correction_inf;
            optimizer.setVerbose(false);
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            //cout << "LocalMap: 10 loops" << endl;
            //remove outliers
            int outlier_cnt = 0;
            int inliers_cnt = 0;
            for(int i=(edges.size()-1); i>=0; i--)
            {
                g2o::EdgeProjectXYZ2UV* e = edges.at(i);
                e->computeError();
                if (e->chi2()>3.0){
                    int id=  e->vertex(0)->id();//outlier landmark id;
                    correction_inf.lm_outlier_id.push_back(id);
                    outlier_cnt++;
                    optimizer.removeEdge(e);
                    edges.erase(edges.begin()+i);
                }else{
                    inliers_cnt++;
                }
            }
            correction_inf.lm_outlier_count=outlier_cnt;
            optimizer.initializeOptimization();
            optimizer.optimize(5);
            //bcout << "LocalMap: 15 loops" << endl;
            //update pose of newest frame
            correction_inf.frame_id=kfs.back().frame_id;

            g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(bag->getNewestPoseInOptimizerIdx()));
            Eigen::Isometry3d pose = v->estimate();
            correction_inf.T_c_w = SE3(pose.rotation(),pose.translation());

            //landmark position
            vector<LM_ITEM> lms;
            bag->getMultiViewLMs(lms,4);
            cout<<"multi view lm  number:  "<<lms.size()<<endl;
            //bag.getAllLMs(lms);
            correction_inf.lm_count = lms.size();
            //cout << "correction_inf" << endl;
            for (auto lm:lms)
            {
                g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(lm.id));
                Eigen::Vector3d pos = v->estimate();
                int64_t id = v->id();
                correction_inf.lm_id.push_back(id);
                correction_inf.lm_3d.push_back(pos);
            }
            optimizer_state=SLIDING_WINDOW;
            pub_correction_inf->pub(correction_inf.frame_id,
                                    correction_inf.T_c_w,
                                    correction_inf.lm_count,
                                    correction_inf.lm_id,
                                    correction_inf.lm_3d,
                                    correction_inf.lm_outlier_count,
                                    correction_inf.lm_outlier_id);

        }//if(optimizer_state==OPTIMIZING)
        kfs.pop_front();
    }//call back function

    virtual void onInit()
    {
        ros::NodeHandle nh = getPrivateNodeHandle();

        string configFilePath;
        nh.getParam("/yamlconfigfile",   configFilePath);
        int cam_type_from_yaml = getIntVariableFromYaml(configFilePath,"type_of_cam");
        if(cam_type_from_yaml==0) cam_type=DEPTH_D435I;
        if(cam_type_from_yaml==1) cam_type=STEREO_EuRoC_MAV;
        if(cam_type==DEPTH_D435I)
        {
            cv::Mat K0 = cameraMatrixFromYamlIntrinsics(configFilePath,"cam0_intrinsics");
            cv::Mat D0   = distCoeffsFromYaml(configFilePath,"cam0_distortion_coeffs");
            fx = K0.at<double>(0,0);
            fy = K0.at<double>(1,1);
            cx = K0.at<double>(0,2);
            cy = K0.at<double>(1,2);
        }
        if(cam_type==STEREO_EuRoC_MAV)
        {
            int w  = getIntVariableFromYaml(configFilePath,"image_width");
            int h = getIntVariableFromYaml(configFilePath,"image_height");
            cv::Mat K0 = cameraMatrixFromYamlIntrinsics(configFilePath,"cam0_intrinsics");
            cv::Mat D0   = distCoeffsFromYaml(configFilePath,"cam0_distortion_coeffs");
            Mat4x4  mat_mavimu_cam0  = Mat44FromYaml(configFilePath,"T_mavimu_cam0");
            SE3 T_mavi_c0 = SE3(mat_mavimu_cam0.topLeftCorner(3,3),
                                mat_mavimu_cam0.topRightCorner(3,1));
            cv::Mat K1 = cameraMatrixFromYamlIntrinsics(configFilePath,"cam1_intrinsics");
            cv::Mat D1   = distCoeffsFromYaml(configFilePath,"cam1_distortion_coeffs");
            Mat4x4  mat_mavimu_cam1  = Mat44FromYaml(configFilePath,"T_mavimu_cam1");
            SE3 T_mavi_c1 = SE3(mat_mavimu_cam1.topLeftCorner(3,3),
                                mat_mavimu_cam1.topRightCorner(3,1));
            SE3 T_c0_c1 = T_mavi_c0.inverse()*T_mavi_c1;
            Mat4x4  mat_i_mavimu  = Mat44FromYaml(configFilePath,"T_imu_mavimu");
            SE3 T_i_mavi = SE3(mat_i_mavimu.topLeftCorner(3,3),mat_i_mavimu.topRightCorner(3,1));
            SE3 T_i_c0 = T_i_mavi*T_mavi_c0;
            Mat3x3 R_=T_c0_c1.inverse().rotation_matrix();
            Vec3   T_=T_c0_c1.inverse().translation();
            cv::Mat R_01 = (cv::Mat1d(3, 3) << R_(0,0), R_(0,1), R_(0,2),
                            R_(1,0), R_(1,1), R_(1,2),
                            R_(2,0), R_(2,1), R_(2,2));
            cv::Mat T_01 = (cv::Mat1d(3, 1) << T_(0), T_(1), T_(2));
            cv::Mat R0,R1,P0,P1,Q;
            cv::stereoRectify(K0,D0,K1,D1,cv::Size(w,h),R_01,T_01,
                              R0,R1,P0,P1,Q,
                              CALIB_ZERO_DISPARITY,0,cv::Size(w,h));

            fx = fy = 435.2616200843788;
            cx = 367.4154319763184;
            cy = 252.1711006164551;
//            fx = P0.at<double>(0,0);
//            fy = P0.at<double>(1,1);
//            cx = P0.at<double>(0,2);
//            cy = P0.at<double>(1,2);

        }
        nh.getParam("window_size",   fix_window_optimizer_size);
        cout << "window_size: " << fix_window_optimizer_size << endl;
        if(fix_window_optimizer_size < 3 || fix_window_optimizer_size > 100)
        {
            fix_window_optimizer_size = 10;
            cout << "invalide window_size use default :" << fix_window_optimizer_size << endl;
        }
        bag = new PoseLMBag(fix_window_optimizer_size);
        optimizer_state = UN_INITIALIZED;

        pub_correction_inf = new CorrectionInfMsg(nh,"/vo_localmap_feedback");

        sub_kf = nh.subscribe<flvis::KeyFrame>(
                    "/vo_kf",
                    10,
                    boost::bind(&LocalMapNodeletClass::frame_callback, this, _1));

    }




};//class LocalMapNodeletClass
}//namespace flvis_ns



PLUGINLIB_EXPORT_CLASS(flvis_ns::LocalMapNodeletClass, nodelet::Nodelet)


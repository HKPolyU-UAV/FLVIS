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
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/csparse/g2o_csparse_api.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


using namespace cv;
using namespace std;

#define FIX_WINDOW_OPTIMIZER_SIZE (8)

static int64_t optimizer_edge_idx;
static int64_t edge_id;


namespace vo_nodelet_ns
{



std::deque<KeyFrameStruct> kfs;
PoseLMBag pose_lm_bag;

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
    g2o::CameraParameters* cam_params;


    void frame_callback(const vo_nodelet::KeyFrameConstPtr& msg)
    {
        KeyFrameStruct kf;
        KeyFrameMsg::unpack(msg,
                            kf.frame_id,
                            kf.img,
                            kf.lm_count,
                            kf.lm_id,
                            kf.lm_2d,
                            kf.lm_3d,
                            kf.lm_descriptor,
                            kf.T_c_w);

        kfs.push_back(kf);
        if(kfs.size()>=FIX_WINDOW_OPTIMIZER_SIZE)//fix window BA
        {
            if(optimizer_initialized)
            {
                //remove outside of window vertex (poses & landmarks) and edges
                //add new vertex and edges

            }
            else//initialize the optimizer
            {

                typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;
                std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverEigen<Block::PoseMatrixType>());
                std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));
                g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));
                g2o::SparseOptimizer optimizer;
                optimizer.setAlgorithm ( solver );


                cout << "-----------------------------------------------------------------------------------------" << endl;
                for(int f_idx = 0; f_idx<FIX_WINDOW_OPTIMIZER_SIZE; f_idx++)//add pose
                {
                    pose_lm_bag.addPose(kfs.at(f_idx).frame_id,kfs.at(f_idx).T_c_w);
                    cout << "lm_cout " << kfs.at(f_idx).lm_count << " " << kfs.at(f_idx).lm_3d.size() << endl;
                    for(int lm_idx=0; lm_idx < kfs.at(f_idx).lm_count; lm_idx++)//add landmarks
                    {
                        pose_lm_bag.addLM(kfs.at(f_idx).lm_id.at(lm_idx),kfs.at(f_idx).lm_3d.at(lm_idx));
                    }
                }
                //add pose vertex;
                vector<POSE_ITEM> poses;
                int oldest_idx1 = pose_lm_bag.getOldestIdx();
                int oldest_idx2 = (oldest_idx1+1);
                if(oldest_idx2 == FIX_WINDOW_OPTIMIZER_SIZE) oldest_idx2 = 0;
                pose_lm_bag.getAllPoses(poses);
                for(std::vector<POSE_ITEM>::iterator it = poses.begin(); it != poses.end(); ++it)
                {
                    g2o::VertexSE3Expmap* v_pose = new g2o::VertexSE3Expmap();
                    v_pose->setId(it->pose_id);//fix first two items
                    if ((it->pose_id==oldest_idx1) || (it->pose_id==oldest_idx2))
                    {
                        v_pose->setFixed(true);
                    }
                    v_pose->setEstimate(g2o::SE3Quat(it->pose.so3().unit_quaternion().toRotationMatrix(),
                                                     it->pose.translation()));
                    optimizer.addVertex(v_pose);
                }
                //add landmark vertex;
                vector<LM_ITEM> lms;
                pose_lm_bag.getAllLMs(lms);
                for(std::vector<LM_ITEM>::iterator it = lms.begin(); it != lms.end(); ++it) {
                    g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
                    point->setId (it->id);
                    point->setEstimate (it->p3d_w);
                    point->setMarginalized ( true );
                    optimizer.addVertex (point);
                }
                pose_lm_bag.debug_output();
                //add camera model
                g2o::CameraParameters* camera = new g2o::CameraParameters(((fx+fy)/2.0), Eigen::Vector2d(cx, cy), 0 );
                camera->setId(0);
                optimizer.addParameter(camera);
                //add edge model
                edge_id = 0;
                vector<g2o::EdgeProjectXYZ2UV*> edges;
                for(int f_idx = 0; f_idx<FIX_WINDOW_OPTIMIZER_SIZE; f_idx++)//add pose
                {
                    int64_t pose_idx = pose_lm_bag.getPoseIdByReleventFrameId(kfs.at(f_idx).frame_id);
                    cout << pose_idx << endl;
                    for(int lm_idx=0; lm_idx < kfs.at(f_idx).lm_count; lm_idx++)//add landmarks
                    {
                        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
                        edge->setId(edge_id);
                        edge_id++;
                        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(kfs.at(f_idx).lm_id.at(lm_idx))));
                        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>   (optimizer.vertex(pose_idx)));
                        edge->setMeasurement( Eigen::Vector2d(kfs.at(f_idx).lm_2d.at(lm_idx)[0], kfs.at(f_idx).lm_2d.at(lm_idx)[1]));
                        edge->setInformation( Eigen::Matrix2d::Identity() );
                        edge->setParameterId(0,0);
                        edge->setRobustKernel(new g2o::RobustKernelHuber());
                        optimizer.addEdge(edge);
                        edges.push_back(edge);
                    }
                }
                optimizer_initialized = true;
                for ( auto e:edges )
                {
                    e->computeError();
                    cout<<"error = "<<e->chi2()<<endl;
                }
                cout<<"start optimization"<<endl;
                optimizer.setVerbose(true);
                optimizer.initializeOptimization();
                optimizer.optimize(10);
                cout<<"end"<<endl;

                //transformation of newest frame
                //                g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
                //                Eigen::Isometry3d pose = v->estimate();
                //                cout<<"Pose="<<endl<<pose.matrix()<<endl;
                //landmark position
                //                for ( size_t i=0; i<20; i++ )
                //                {
                //                    g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
                //                    cout<<"vertex id "<<i+2<<", pos = ";
                //                    Eigen::Vector3d pos = v->estimate();
                //                    cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
                //                }
                //calculatie inliers
                int inliers = 0;
                for ( auto e:edges )
                {
                    e->computeError();
                    if (e->chi2()>3.0)
                    {
                        cout<<"error = "<<e->chi2()<<endl;
                    }
                    else
                    {
                        inliers++;
                    }
                }
                cout << inliers << endl;
            }
            if(optimizer_initialized)
            {//dealing with the output

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


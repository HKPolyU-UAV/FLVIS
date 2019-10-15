#include "include/bundle_adjustment.h"



bundleAdjustment::bundleAdjustment()
{

}

void bundleAdjustment::BAInFrame(CameraFrame &frame)
{
    //get all landmarks (has depth information and is inliers)
    double fx=frame.d_camera.camera_fx;
    double fy=frame.d_camera.camera_fy;
    double cx=frame.d_camera.camera_cx;
    double cy=frame.d_camera.camera_cy;
    vector<LandMarkInFrame> lms_in_frame;
    frame.getValidInliersPair(lms_in_frame);
    cout << lms_in_frame.size() << "|" << frame.landmarks.size() << endl;
    if(lms_in_frame.size()<10)
    {
        return;
    }
    else {
        g2o::CameraParameters* cam_params;
        typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;
        std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverEigen<Block::PoseMatrixType>());
        std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm (solver);

        g2o::CameraParameters* camera = new g2o::CameraParameters(((fx+fy)/2.0), Eigen::Vector2d(cx, cy), 0 );
        camera->setId(0);
        optimizer.addParameter(camera);

        //add pose vertex
        g2o::VertexSE3Expmap* v_pose = new g2o::VertexSE3Expmap();
        v_pose->setId(0);
        v_pose->setEstimate(g2o::SE3Quat(frame.T_c_w.so3().unit_quaternion().toRotationMatrix(),
                                         frame.T_c_w.translation()));
        optimizer.addVertex(v_pose);

        vector<g2o::EdgeProjectXYZ2UV*> edges;
        for(auto lm : lms_in_frame)
        {
            g2o::VertexSBAPointXYZ* v_point = new g2o::VertexSBAPointXYZ();
            v_point->setId (lm.lm_id);
            v_point->setEstimate (lm.lm_3d_w);
            v_point->setMarginalized (true);
            //v_point->setFixed(true);
            optimizer.addVertex (v_point);
            g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
            edge->setId(lm.lm_id);
            edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(lm.lm_id)));
            edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>   (optimizer.vertex(0)));
            edge->setMeasurement(Eigen::Vector2d(lm.lm_2d));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setParameterId(0,0);
            edge->setRobustKernel(new g2o::RobustKernelHuber());
            optimizer.addEdge(edge);
            edges.push_back(edge);
        }
        cout<<"start optimization"<<endl;
        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.optimize(2);
        cout<<"end"<<endl;
        g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(0) );
        Eigen::Isometry3d pose = v->estimate();

        //update frame pose
        frame.T_c_w =  SE3(pose.rotation(),pose.translation());
        //update landmarks
//        for(auto e:edges)
//        {
//            if(e->chi2()<2.0)
//            {
//                uint64_t id=e->id();
//                for(int i=0; i<frame.landmarks.size(); i++)
//                {
//                    if(frame.landmarks.at(i).lm_id == id)
//                    {
//                        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(frame.landmarks.at(i).lm_id));
//                        Eigen::Vector3d pos = v->estimate();
//                        frame.landmarks.at(i).lm_3d_w = pos;
//                        frame.landmarks.at(i).lm_3d_c = DepthCamera::world2cameraT_c_w(pos,frame.T_c_w);
//                        break;
//                    }
//                }
//            }
//        }
    }
}

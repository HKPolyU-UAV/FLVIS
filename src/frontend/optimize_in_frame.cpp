#include "include/optimize_in_frame.h"



OptimizeInFrame::OptimizeInFrame()
{

}

void OptimizeInFrame::optimize(CameraFrame &frame)
{
    //get all landmarks (has depth information and is inliers)
    double fx=frame.d_camera.cam0_fx;
    double fy=frame.d_camera.cam0_fy;
    double cx=frame.d_camera.cam0_cx;
    double cy=frame.d_camera.cam0_cy;
    vector<LandMarkInFrame> lms_in_frame;
    frame.getValidInliersPair(lms_in_frame);
//    cout << lms_in_frame.size() << "|" << frame.landmarks.size() << endl;
    if(lms_in_frame.size()<10)
    {
        return;
    }
    else {
        typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;
        std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverEigen<Block::PoseMatrixType>());
        std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm (solver);

        //add pose vertex
        g2o::VertexSE3Expmap* v_pose = new g2o::VertexSE3Expmap();
        v_pose->setId(0);
        v_pose->setEstimate(g2o::SE3Quat(frame.T_c_w.so3().unit_quaternion().toRotationMatrix(),
                                         frame.T_c_w.translation()));
        optimizer.addVertex(v_pose);

        vector<g2o::EdgeSE3ProjectXYZ*> edges;
        for(auto lm : lms_in_frame)
        {
            g2o::VertexSBAPointXYZ* v_point = new g2o::VertexSBAPointXYZ();
            v_point->setId (lm.lm_id);
            v_point->setEstimate (lm.lm_3d_w);
            v_point->setFixed(true);
            optimizer.addVertex (v_point);
            g2o::EdgeSE3ProjectXYZ* edge = new g2o::EdgeSE3ProjectXYZ();
            edge->fx = fx;
            edge->fy = fy;
            edge->cx = cx;
            edge->cy = cy;
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
        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.optimize(2);
        for (auto e:edges)
        {
            e->computeError();
            if (e->chi2()>3.0){
                optimizer.removeEdge(e);
                e->id();
            }
        }
        optimizer.initializeOptimization();
        optimizer.optimize(2);
//        optimizer.initializeOptimization();
//        optimizer.optimize(2);
//        cout<<"end"<<endl;
        g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(0) );
        Eigen::Isometry3d pose = v->estimate();

        //update frame pose
        frame.T_c_w =  SE3(pose.rotation(),pose.translation());

    }
}

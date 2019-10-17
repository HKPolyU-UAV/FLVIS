#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <deque>
#include <stdint.h>

#include <include/yamlRead.h>

#include <include/keyframe_msg.h>
#include <vo_nodelet/KeyFrame.h>
#include <geometry_msgs/Vector3.h>

// DBoW3
#include "../3rdPartLib/DBow3/src/DBoW3.h"
#include "../3rdPartLib/DBow3/src/DescManip.h"
#include "../3rdPartLib/DBow3/src/Vocabulary.h"
#include "../3rdPartLib/DBow3/src/BowVector.h"
#include "../3rdPartLib/DBow3/src/ScoringObject.h"
#include "../3rdPartLib/DBow3/src/Database.h"
//g2o
#include <g2o/config.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/sba/types_six_dof_expmap.h>


//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>

//#include <pcl/common/transforms.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <condition_variable>
#include <octomap/octomap.h>




using namespace DBoW3;
using namespace cv;
using namespace std;

static uint64_t kf_id;

namespace vo_nodelet_ns
{


struct sort_simmat_by_score
{
    inline bool operator()(const Vector2d& a, const Vector2d& b){
        return ( a(1) > b(1) );
    }
};
struct sort_descriptor_by_queryIdx
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ( a[0].queryIdx < b[0].queryIdx );
    }
};

#define lcKFDist (6)
#define lcKFMaxDist (50)
#define lcKFLast (20)
#define lcNKFClosest (3)
#define ratioMax (0.7)
#define ratioRansac (0.6)
#define minPts (30)


class VOLoopClosingNodeletClass : public nodelet::Nodelet
{
public:
    VOLoopClosingNodeletClass()  {;}
    ~VOLoopClosingNodeletClass() {;}

private:
    ros::Subscriber sub_kf;
    int image_width,image_height;
    Mat cameraMatrix,distCoeffs;
    Mat diplay_img;
    double fx,fy,cx,cy;
    bool optimizer_initialized;

    vector<int64_t> optimizer_lm_id;

    //DBow related para
    Vocabulary voc;
    Database db;// faster search
    vector<vector<double>> sim_matrix;//for similarity visualization
    //KF database
    vector<shared_ptr<KeyFrameStruct>> kf_map;
    vector<BowVector> kfbv_map;
    //loop info
    vector<Vec3I> loop_ids;
    vector<SE3> loop_poses;
    //uint64_t kf_prev_idx, kf_curr_idx;






    bool isLoopCandidate( uint64_t &kf_prev_idx)
    {
      bool is_lc_candidate = false;
      int g_size = kf_map.size();
      vector<Vector2d> max_sim_mat;
      if(g_size < 40) return is_lc_candidate;
      for (uint64_t i = 0; i < (uint64_t)(g_size - lcKFDist); i++)
      {
          if (kf_map[i] != nullptr)
          {
              Vector2d aux;
              aux(0) = i;
              aux(1) = sim_matrix[i][g_size-1];
              max_sim_mat.push_back(aux);
          }
      }

      sort(max_sim_mat.begin(), max_sim_mat.end(), sort_simmat_by_score());

      // find the minimum score in the covisibility graph (and/or 3 previous keyframes)
      double lc_min_score = 1.0;
      for (uint64_t i = (uint64_t)(g_size - lcKFDist); i < (uint64_t)(g_size); i++)
      {
          double score_i = sim_matrix[i][g_size-1];
          if (score_i < lc_min_score && score_i > 0.001) lc_min_score = score_i;
      }

      int idx_max = int(max_sim_mat[0](0));
      int nkf_closest = 0;
      if (max_sim_mat[0](1) >= lc_min_score)
      {
        cout<<"may be a loop: "<<endl;
        // there must be at least lc_nkf_closest KFs conected to the LC candidate with a score above lc_dbow_score_min
        for (uint64_t i = 1; i < max_sim_mat.size(); i++)
        {
          int idx = int(max_sim_mat[i](0));

          if (abs(idx - idx_max) <= lcKFMaxDist && max_sim_mat[i](1) >= lc_min_score * 0.8) nkf_closest++;
        }
       }

       // update in case of being loop closure candidate
       if (nkf_closest >= lcNKFClosest)
       {
           is_lc_candidate = true;
           kf_prev_idx = idx_max;
       }

       // ****************************************************************** //
       cout << endl
            << "lc_min_score: " << lc_min_score;
       cout << endl
            << "Nkf_closest:  " << nkf_closest;
       cout << endl
            << "idx_max:  " << idx_max << endl;
       cout << "max score of previous kfs: "<<max_sim_mat[0](1)<<endl;
       return is_lc_candidate;
    }

    bool isLoopClosure(shared_ptr<KeyFrameStruct> kf0, shared_ptr<KeyFrameStruct> kf1,SE3 &se_ji)
    {
      //kf0 previous kf, kf1 current kf,
      bool is_lc = false;
      int common_pt = 0;

      if (!(kf1->lm_descriptor.size() == 0) && !(kf0->lm_descriptor.size() == 0))
      {

          BFMatcher *bfm = new BFMatcher(NORM_HAMMING, false); // cross-check
          Mat pdesc_l1= Mat::zeros(Size(32,kf0->lm_descriptor.size()),CV_8U);
          Mat pdesc_l2= Mat::zeros(Size(32,kf1->lm_descriptor.size()),CV_8U);
          vector<vector<DMatch>> pmatches_12, pmatches_21;
          // 12 and 21 matches
          vMat_to_descriptors(pdesc_l1,kf0->lm_descriptor);
          vMat_to_descriptors(pdesc_l2,kf1->lm_descriptor);
          cout<<"size: "<<pdesc_l1.size().height<<" "<<pdesc_l1.size().width<<endl;
          cout<<"size: "<<pdesc_l2.size().height<<" "<<pdesc_l2.size().width<<endl;
          bfm->knnMatch(pdesc_l1, pdesc_l2, pmatches_12, 2);
          bfm->knnMatch(pdesc_l2, pdesc_l1, pmatches_21, 2);

          // resort according to the queryIdx
          sort(pmatches_12.begin(), pmatches_12.end(), sort_descriptor_by_queryIdx());
          sort(pmatches_21.begin(), pmatches_21.end(), sort_descriptor_by_queryIdx());

          // bucle around pmatches

          vector<Point3f> p3d;
          vector<Point2f> p2d;
          p3d.clear();
          p2d.clear();

          for (size_t i = 0; i < pmatches_12.size(); i++)
          {
              // check if they are mutual best matches
              int lr_qdx = pmatches_12[i][0].queryIdx;
              int lr_tdx = pmatches_12[i][0].trainIdx;
              int rl_tdx = pmatches_21[lr_tdx][0].trainIdx;

              // check if they are mutual best matches and satisfy the distance ratio test

              if (lr_qdx == rl_tdx)
              {
                  if(pmatches_12[i][0].distance/pmatches_12[i][1].distance < ratioMax)
                  {
                    common_pt++;
                    // save data for optimization
                    Vector3d P = kf0->lm_3d[lr_qdx];
                    Vector2d pl_obs = kf1->lm_2d[lr_tdx];
                    Point3f p3(P(0),P(1),P(2));
                    Point2f p2(pl_obs(0),pl_obs(1));
                    p3d.push_back(p3);
                    p2d.push_back(p2);
                  }

              }

          }
          cv::Mat r_ = cv::Mat::zeros(3, 1, CV_64FC1);
          cv::Mat t_ = cv::Mat::zeros(3, 1, CV_64FC1);
          Mat inliers;
          SE3_to_rvec_tvec(kf0->T_c_w, r_ , t_ );
          solvePnPRansac(p3d,p2d,cameraMatrix,distCoeffs,r_,t_,false,100,3.0,0.99,inliers,SOLVEPNP_P3P);
          cout<<"selected points size: "<<p3d.size()<<" inliers size: "<<inliers.rows<<" unseletced size: "<<pmatches_12.size()<<endl;


          if(inliers.rows/p3d.size() < ratioRansac || inliers.rows < minPts ) return is_lc;

          SE3 se_iw = kf0->T_c_w;
          SE3 se_jw = SE3_from_rvec_tvec(r_,t_);
          SE3 se_jw_p = kf1->T_c_w;

          se_ji = se_jw*se_iw.inverse();// i previous kf, j current kf, sji = sjw*swi; from prev to curr
          //se_ij = se_ji.inverse();
          SE3 se_j_correct = se_jw*se_jw_p.inverse();

          is_lc = true;
      }
      return is_lc;
    }



    void expandGraph()
    {
      int g_size = (int) kfbv_map.size();

      // sim matrix
      sim_matrix.resize(g_size);
      for (int i = 0; i < g_size; i++)
          sim_matrix[i].resize(g_size);

    }

    //noticed g2o pgo use swi swj as vertices, where swi transform p from i to w
    //use sij as edge
    //error function is sij^-1 * (swi^-1*swj)
    //so we need to use Tcw.inverse() as vertices, and sji.inverse as edge
    void loopClosureOnEssentialGraphG2O()
    {
      uint64_t kf_prev_idx = 2 * kf_map.size();
      uint64_t kf_curr_idx = 0;
      for(size_t i = 0; i < loop_ids.size(); i++)
      {
        if(loop_ids[i](0) < kf_prev_idx)
          kf_prev_idx = loop_ids[i](0);
        if(loop_ids[i](1) > kf_curr_idx)
          kf_curr_idx = loop_ids[i](1);
      }
      cout<<"first and last id in the loop: "<<kf_prev_idx<<" "<<kf_curr_idx<<endl;


      g2o::SparseOptimizer optimizer;
      optimizer.setVerbose(true);


      std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>());
      std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));
      g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

      solver->setUserLambdaInit(1e-10);
      optimizer.setAlgorithm(solver);

      // grab the KFs included in the optimization
      vector<int> kf_list;
      for (int i = kf_prev_idx; i <= kf_curr_idx; i++)
      {
          if (kf_map[i] != nullptr)
          {
              // check if it is a LC vertex
              bool is_lc_i = false;
              bool is_lc_j = false;
              int id = 0;
              for (auto it = loop_ids.begin(); it != loop_ids.end(); it++, id++)
              {
                  if ((*it)(0) == i)
                  {
                      is_lc_i = true;
                      break;
                  }
                  if ((*it)(1) == i)
                  {
                      is_lc_j = true;
                      break;
                  }
              }
              kf_list.push_back(i);
              // create SE3 vertex
              g2o::VertexSE3 *v_se3 = new g2o::VertexSE3();
              v_se3->setId(i);
              v_se3->setMarginalized(false);

              if (is_lc_j)
              {
                  // update pose of LC vertex
                  v_se3->setFixed(false);
                  SE3 tmp = kf_map[i]->T_c_w.inverse();
                  v_se3->setEstimate(SE3_to_g2o(tmp));
              }
              else
              {
                  SE3 tmp = kf_map[i]->T_c_w.inverse();
                  v_se3->setEstimate(SE3_to_g2o(tmp));
                  //if ((is_lc_i && loop_ids.back()[0] == i) || i == 0)
                  if(i == 0)
                      v_se3->setFixed(true);
                  else
                      v_se3->setFixed(false);
              }
              optimizer.addVertex(v_se3);
          }
      }


      // introduce edges
      for (int i = kf_prev_idx; i <= kf_curr_idx; i++)
      {
          for (int j = i + 1; j <= kf_curr_idx; j++)
          {
              if (kf_map[i] != nullptr && kf_map[j] != nullptr)
              {
                  // kf2kf constraint
                  SE3 sji = kf_map[j]->T_c_w*kf_map[i]->T_c_w.inverse();//se_jw*se_iw.inverse();
                  // add edge
                  // _inverseMeasurement * from->estimate().inverse() * to->estimate();
                  // z^-1 * (x_i^-1 * x_j)
                  // error sij^-1 * (swi)^-1 * swj
                  //
                  SE3 sij = sji.inverse();
                  g2o::EdgeSE3* e_se3 = new g2o::EdgeSE3();
                  //g2o::EdgeSE3 *e_se3 = new g2o::EdgeSE3();
                  e_se3->setVertex(0, optimizer.vertex(i));
                  e_se3->setVertex(1, optimizer.vertex(j));
                  e_se3->setMeasurement(SE3_to_g2o(sij));
                  //e_se3->information() = map_keyframes[j]->xcov_kf_w;
                  e_se3->setInformation(Matrix6d::Identity());
                  optimizer.addEdge(e_se3);
              }
          }
      }

      // introduce loop closure edges
      int id = 0;
      for (auto it = loop_ids.begin(); it != loop_ids.end(); it++, id++)
      {
          // add edge
          g2o::EdgeSE3 *e_se3 = new g2o::EdgeSE3();
          e_se3->setVertex(0, optimizer.vertex((*it)(0)));
          e_se3->setVertex(1, optimizer.vertex((*it)(1)));
          SE3 loop_pose = loop_poses[id].inverse();
          e_se3->setMeasurement(SE3_to_g2o(loop_pose));
          e_se3->information() = Matrix6d::Identity();
          optimizer.addEdge(e_se3);
      }


      // optimize graph
      optimizer.initializeOptimization();
      optimizer.computeInitialGuess();
      optimizer.computeActiveErrors();
      optimizer.optimize(100);

      // recover pose and update map
      // Matrix4d Tkfw_corr;
      for (auto kf_it = kf_list.begin(); kf_it != kf_list.end(); kf_it++)
      {
          g2o::VertexSE3 *v_se3 = static_cast<g2o::VertexSE3 *>(optimizer.vertex((*kf_it)));
          g2o::SE3Quat Twc_g2o = v_se3->estimateAsSE3Quat();
          SE3 Tcw_prev = kf_map[(*kf_it)]->T_c_w;
          SE3 Twc_pgo = SE3_from_g2o(Twc_g2o);
          SE3 Tcw_corr =Twc_pgo.inverse()*Tcw_prev.inverse();//Tpgo_prev =

          //kf_map[(*kf_it)]->T_c_w = Tcw_pgo.inverse();
          //cout<<"after g2o pgo: ";
          //cout<<Tcw_corr<<endl;



      }












    }
    void loopClosureOnCovGraphG2O()
    {
      uint64_t kf_prev_idx = 2 * kf_map.size();
      uint64_t kf_curr_idx = 0;
      for(size_t i = 0; i < loop_ids.size(); i++)
      {
        if(loop_ids[i](0) < kf_prev_idx)
          kf_prev_idx = loop_ids[i](0);
        if(loop_ids[i](1) > kf_curr_idx)
          kf_curr_idx = loop_ids[i](1);
      }
      cout<<"first and last id in the loop: "<<kf_prev_idx<<" "<<kf_curr_idx<<endl;


      g2o::SparseOptimizer optimizer;
      optimizer.setVerbose(true);


      std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>());
      std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver)));
      g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

      solver->setUserLambdaInit(1e-10);
      optimizer.setAlgorithm(solver);

      //noticed g2o pgo use swi swj as vertices, where swi transform p from i to w
      //use sij as edge
      //error function is sij^-1 * (swi^-1*swj)
      //so we need to use Tcw.inverse() as vertices, and sji.inverse as edge

      // grab the KFs included in the optimization
      vector<int> kf_list;
      for (int i = kf_prev_idx; i <= kf_curr_idx; i++)
      {
          if (kf_map[i] != nullptr)
          {
              // check if it is a LC vertex
              bool is_lc_i = false;
              bool is_lc_j = false;
              int id = 0;
              for (auto it = loop_ids.begin(); it != loop_ids.end(); it++, id++)
              {
                  if ((*it)(0) == i)
                  {
                      is_lc_i = true;
                      break;
                  }
                  if ((*it)(1) == i)
                  {
                      is_lc_j = true;
                      break;
                  }
              }
              kf_list.push_back(i);
              // create SE3 vertex
              g2o::VertexSE3 *v_se3 = new g2o::VertexSE3();
              v_se3->setId(i);
              v_se3->setMarginalized(false);

              // _inverseMeasurement * from->estimate().inverse() * to->estimate();
              // z^-1 * (x_i^-1 * x_j)
              SE3 siw = kf_map[i]->T_c_w.inverse();

              if (is_lc_j)
              {
                  // update pose of LC vertex
                  v_se3->setFixed(false);

                  v_se3->setEstimate(SE3_to_g2o(siw));
              }
              else
              {
                  v_se3->setEstimate(SE3_to_g2o(siw));
                  //if ((is_lc_i && loop_ids.back()[0] == i) || i == 0)
                  if(i == 0)
                      v_se3->setFixed(true);
                  else
                      v_se3->setFixed(false);
              }
              optimizer.addVertex(v_se3);
          }
      }


      // introduce edges
      for (int i = kf_prev_idx; i <= kf_curr_idx; i++)
      {
          for (int j = i + 1; j <= kf_curr_idx, j <= kf_prev_idx +5 ; j++)
          {
              if (kf_map[i] != nullptr && kf_map[j] != nullptr)
              {
                  // kf2kf constraint
                  SE3 sji = kf_map[j]->T_c_w*kf_map[i]->T_c_w.inverse();//se_jw*se_iw.inverse();
                  SE3 sij = sji.inverse();
                  // add edge
                  g2o::EdgeSE3* e_se3 = new g2o::EdgeSE3();
                  //g2o::EdgeSE3 *e_se3 = new g2o::EdgeSE3();
                  e_se3->setVertex(0, optimizer.vertex(i));
                  e_se3->setVertex(1, optimizer.vertex(j));
                  e_se3->setMeasurement(SE3_to_g2o(sij));
                  //e_se3->information() = map_keyframes[j]->xcov_kf_w;
                  e_se3->setInformation(Matrix6d::Identity());
                  optimizer.addEdge(e_se3);
              }
          }
      }

      // introduce loop closure edges
      int id = 0;
      for (auto it = loop_ids.begin(); it != loop_ids.end(); it++, id++)
      {
          // add edge
          g2o::EdgeSE3 *e_se3 = new g2o::EdgeSE3();
          e_se3->setVertex(0, optimizer.vertex((*it)(0)));
          e_se3->setVertex(1, optimizer.vertex((*it)(1)));
          SE3 loop_pose = loop_poses[id].inverse();
          e_se3->setMeasurement(SE3_to_g2o(loop_pose));
          e_se3->information() = Matrix6d::Identity();
          optimizer.addEdge(e_se3);
      }


      // optimize graph
      optimizer.initializeOptimization();
      optimizer.computeInitialGuess();
      optimizer.computeActiveErrors();
      optimizer.optimize(100);

      // recover pose and update map
      // Matrix4d Tkfw_corr;
      for (auto kf_it = kf_list.begin(); kf_it != kf_list.end(); kf_it++)
      {
          g2o::VertexSE3 *v_se3 = static_cast<g2o::VertexSE3 *>(optimizer.vertex((*kf_it)));
          g2o::SE3Quat Twc_g2o = v_se3->estimateAsSE3Quat();
          SE3 Tcw_prev = kf_map[(*kf_it)]->T_c_w;
          SE3 Twc_pgo = SE3_from_g2o(Twc_g2o);
          SE3 Tcw_corr =Twc_pgo.inverse()*Tcw_prev.inverse();//Tpgo_prev =

          //kf_map[(*kf_it)]->T_c_w = Twc_pgo.inverse();
          //cout<<"after g2o pgo: ";
          //cout<<Tcw_corr<<endl;



      }



    }










    void frame_callback(const vo_nodelet::KeyFrameConstPtr& msg)
    {


        KeyFrameStruct kf;
        BowVector kf_bv;
        SE3 loop_pose;
        KeyFrameMsg::unpack(msg,kf.frame_id,kf.img,kf.lm_count,kf.lm_id,kf.lm_2d,kf.lm_3d,kf.lm_descriptor,kf.T_c_w);
        shared_ptr<KeyFrameStruct> kf_ptr = make_shared<KeyFrameStruct>(kf);

        voc.transform(kf_ptr->lm_descriptor,kf_bv);
        kf_map.push_back(kf_ptr);
        kfbv_map.push_back(kf_bv);
        expandGraph();
        for (uint64_t i = 0; i < kf_map.size(); i++)
        {
          if(kf_map[i] != nullptr)
          {
            double score = voc.score(kf_bv,kfbv_map[i]);
            sim_matrix[kfbv_map.size()-1][i] = score;
            sim_matrix[i][kfbv_map.size()-1] = score;
          }
        }
        uint64_t kf_prev_idx;
        bool is_lc_candidate = isLoopCandidate(kf_prev_idx);
        bool is_lc = false;
        uint64_t kf_curr_idx = kf_map.size()-1;
        //if(!is_lc_candidate) return;
        //just test loop pose estimation
        if(kf_curr_idx > 20) bool is_lc = isLoopClosure(kf_map[kf_curr_idx-5],kf_map[kf_curr_idx],loop_pose);
        {
          loop_ids.push_back(Vec3I(kf_curr_idx-5, kf_curr_idx, 1));
          loop_poses.push_back(loop_pose);
          loopClosureOnCovGraphG2O();
        }

        // real loop clpsure correction, but need to tune paras
//        if(kf_curr_idx > 10) is_lc = isLoopClosure(kf_map[kf_prev_idx], kf_map[kf_curr_idx], loop_pose);
//        if(!is_lc) return;
//        if(is_lc)
//        {
//          loop_ids.push_back(Vec3I(kf_prev_idx, kf_curr_idx, 1));
//          loop_poses.push_back(loop_pose);
//          loopClosureOnCovGraphG2O();
//        }















        //imshow("loopclosing", img);
        //waitKey(1);

        cout << endl << "Loop closing Callback" << endl;
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
        string vocFile;
        nh.getParam("/voc", vocFile);
        Vocabulary vocTmp(vocFile);
        voc = vocTmp;
        kf_id = 0;
        Database dbTmp(voc, true, 0);
        db = dbTmp;




        sub_kf = nh.subscribe<vo_nodelet::KeyFrame>(
                    "/vo_kf",
                    10,
                    boost::bind(&VOLoopClosingNodeletClass::frame_callback, this, _1));

    }




};//class VOLoopClosingNodeletClass
}//namespace vo_nodelet_ns



PLUGINLIB_EXPORT_CLASS(vo_nodelet_ns::VOLoopClosingNodeletClass, nodelet::Nodelet)


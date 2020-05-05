#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <deque>
#include <stdint.h>

#include <include/yamlRead.h>

#include <include/keyframe_msg.h>
#include <flvis/KeyFrame.h>
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


#include <include/tic_toc_ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <condition_variable>
#include <octomap/octomap.h>
#include <octomap/OccupancyOcTreeBase.h>

// For TF

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <include/rviz_path.h>

using namespace DBoW3;
using namespace std;


namespace flvis_ns
{


struct sort_simmat_by_score
{
    inline bool operator()(const Vector2d& a, const Vector2d& b){
        return ( a(1) > b(1) );
    }
};
struct sort_descriptor_by_queryIdx
{
    inline bool operator()(const vector<cv::DMatch>& a, const vector<cv::DMatch>& b){
        return ( a[0].queryIdx < b[0].queryIdx );
    }
};

#define lcKFStart (25)
#define lcKFDist (18)
#define lcKFMaxDist (50)
#define lcKFLast (20)
#define lcNKFClosest (2)
#define ratioMax (0.5)
#define ratioRansac (0.5)
#define minPts (20)
#define minScore (0.12)
struct KeyFrameLC
{
  int64_t         frame_id;
  int64_t         keyframe_id;
  int             lm_count;
  vector<Vec2>    lm_2d;
  vector<double>  lm_d;
  vector<cv::Mat>     lm_descriptor;
  BowVector       kf_bv;
  SE3             T_c_w_odom;
  SE3             T_c_w;
  ros::Time       t;
};

struct KeyFrameLCStruct
{
  int64_t         frame_id;
  int64_t         keyframe_id;
  int             lm_count;
  vector<int64_t> lm_id;
  vector<Vec2>    lm_2d;
  vector<Vec3  >  lm_3d;
  vector<cv::Mat>     lm_descriptor;
  BowVector       kf_bv;
  SE3             T_c_w_odom;
  SE3             T_c_w;
  ros::Time       t;
};


class LoopClosingNodeletClass : public nodelet::Nodelet
{
public:
    LoopClosingNodeletClass()  {;}
    ~LoopClosingNodeletClass() {;}

private:
    ros::Subscriber sub_kf;
    int image_width,image_height;
    cv::Mat cameraMatrix,distCoeffs;
    cv::Mat diplay_img;
    double fx,fy,cx,cy;
    enum TYPEOFCAMERA cam_type;
    //bool optimizer_initialized;
    
    ros::Time tt;
    tf::Transform transform;
    tf::TransformBroadcaster br;

    vector<int64_t> optimizer_lm_id;

    //DBow related para
    Vocabulary voc;
    Database db;// faster search
    vector<vector<double>> sim_matrix;//for similarity visualization
    vector<double> sim_vec;
    //KF database
    //vector<shared_ptr<KeyFrameStruct>> kf_map;
    vector<BowVector> kfbv_map;
    vector<shared_ptr<KeyFrameLC>> kf_map_lc;
    //loop info
    vector<Vec3I> loop_ids;
    vector<SE3> loop_poses;
    //uint64_t kf_prev_idx, kf_curr_idx;

    SE3 T_odom_map = SE3();
    SE3 T_prevmap_map = SE3();
    vector<SE3> vmap_correct;
    RVIZPath* path_lc_pub;

    //kf id
    int64_t kf_id = 0;
    //last loop id
    int64_t last_pgo_id = -1000;





    bool isLoopCandidate( uint64_t &kf_prev_idx)
    {
      cout<<"start to find loop candidate."<<endl;
      bool is_lc_candidate = false;
      size_t g_size = kf_map_lc.size();
      //cout<<"kf size: "<<g_size<<endl;
      vector<Vector2d> max_sim_mat;
      if(g_size < 40) return is_lc_candidate;
      for (uint64_t i = 0; i < static_cast<uint64_t>(g_size - lcKFDist); i++)
      {
          if (kf_map_lc[i] != nullptr)
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
      for (uint64_t i = static_cast<uint64_t>(g_size - lcKFDist); i < static_cast<uint64_t>(g_size); i++)
      {
          double score_i = sim_matrix[i][g_size-1];
          if (score_i < lc_min_score && score_i > 0.001) lc_min_score = score_i;
      }

      lc_min_score = min(lc_min_score, 0.4);
      cout<<"max sim score is: "<< max_sim_mat[0](1)<<endl;
      if( max_sim_mat[0](1) < max(minScore, lc_min_score)) return is_lc_candidate;

      int idx_max = int(max_sim_mat[0](0));
      int nkf_closest = 0;
      if (max_sim_mat[0](1) >= lc_min_score)
      {
        // there must be at least lc_nkf_closest KFs conected to the LC candidate with a score above lc_dbow_score_min
        for (uint64_t i = 1; i < max_sim_mat.size(); i++)
        {
          int idx = int(max_sim_mat[i](0));

          if (abs(idx - idx_max) <= lcKFMaxDist && max_sim_mat[i](1) >= lc_min_score * 0.8) nkf_closest++;
        }
       }


       // update in case of being loop closure candidate
       if (nkf_closest >= lcNKFClosest && max_sim_mat[0](1) > minScore)
       {
           is_lc_candidate = true;
           kf_prev_idx = static_cast<uint64_t>(idx_max);
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

    bool isLoopClosure(shared_ptr<KeyFrameLCStruct> kf0, shared_ptr<KeyFrameLCStruct> kf1,SE3 &se_ji)
    {
      //kf0 previous kf, kf1 current kf,
      bool is_lc = false;
      int common_pt = 0;

      if (!(kf1->lm_descriptor.size() == 0) && !(kf0->lm_descriptor.size() == 0))
      {

          cv::BFMatcher *bfm = new cv::BFMatcher(cv::NORM_HAMMING, false); // cross-check
          cv::Mat pdesc_l1= cv::Mat::zeros(cv::Size(32,static_cast<int>(kf0->lm_descriptor.size())),CV_8U);
          cv::Mat pdesc_l2= cv::Mat::zeros(cv::Size(32,static_cast<int>(kf1->lm_descriptor.size())),CV_8U);
          vector<vector<cv::DMatch>> pmatches_12, pmatches_21;
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

          vector<cv::Point3f> p3d;
          vector<cv::Point2f> p2d;
          p3d.clear();
          p2d.clear();

          for (size_t i = 0; i < pmatches_12.size(); i++)
          {
              // check if they are mutual best matches
              size_t lr_qdx = static_cast<size_t>(pmatches_12[i][0].queryIdx);
              size_t lr_tdx = static_cast<size_t>(pmatches_12[i][0].trainIdx);
              size_t rl_tdx = static_cast<size_t>(pmatches_21[lr_tdx][0].trainIdx);

              // check if they are mutual best matches and satisfy the distance ratio test

              if (lr_qdx == rl_tdx)
              {
                  if(pmatches_12[i][0].distance/pmatches_12[i][1].distance < static_cast<float>(ratioMax))
                  {
                    common_pt++;
                    // save data for optimization
                    Vector3f P = (kf0->lm_3d[lr_qdx]).cast <float> ();
                    Vector2f pl_obs = (kf1->lm_2d[lr_tdx]).cast <float> ();

                    cv::Point3f p3(P(0),P(1),P(2));
                    cv::Point2f p2(pl_obs(0),pl_obs(1));
                    p3d.push_back(p3);
                    p2d.push_back(p2);
                  }

              }

          }
          cv::Mat r_ = cv::Mat::zeros(3, 1, CV_64FC1);
          cv::Mat t_ = cv::Mat::zeros(3, 1, CV_64FC1);
          cv::Mat inliers;
          SE3_to_rvec_tvec(kf0->T_c_w, r_ , t_ );
          if(p3d.size() < 5) return is_lc;
          solvePnPRansac(p3d,p2d,cameraMatrix,distCoeffs,r_,t_,false,100,3.0,0.99,inliers,cv::SOLVEPNP_P3P);
          cout<<"selected points size: "<<p3d.size()<<" inliers size: "<<inliers.rows<<" unseletced size: "<<pmatches_12.size()<<endl;
          cout<<"ratio test: "<<inliers.rows*1.0/p3d.size()<<" "<<"number test "<<inliers.rows<<endl;

          if(inliers.rows*1.0/p3d.size() < ratioRansac || inliers.rows < minPts ) //return is_lc;
          {
            cout<<"dont believe: "<<endl;
            cout<<"ratio test: "<<static_cast<double>(inliers.rows)/static_cast<double>(p3d.size())<<" "<<"number test "<<inliers.rows<<endl;
            return is_lc;
          }

          cout<<"after ransac test: "<<endl;

          SE3 se_iw = kf0->T_c_w;
          SE3 se_jw = SE3_from_rvec_tvec(r_,t_);
          //SE3 se_jw_p = kf1->T_c_w;

          se_ji = se_jw*se_iw.inverse();// i previous kf, j current kf, sji = sjw*swi; from prev to curr
          //se_ij = se_ji.inverse();
          //SE3 se_j_correct = se_jw*se_jw_p.inverse();

          if(se_ji.translation().norm() < 3 && se_ji.so3().log().norm() < 1.5) is_lc = true;
      }
      return is_lc;
    }

    bool isLoopClosureKF(shared_ptr<KeyFrameLC> kf0, shared_ptr<KeyFrameLC> kf1,SE3 &se_ji)
    {
      //kf0 previous kf, kf1 current kf,
      bool is_lc = false;
      int common_pt = 0;
      cout<<"kf1 des size: "<<kf1->lm_descriptor.size()<<"kf 0 des size: "<<kf0->lm_descriptor.size()<<endl;

      if (!(kf1->lm_descriptor.size() == 0) && !(kf0->lm_descriptor.size() == 0))
      {
        cout<<"feature ,matching:"<<endl;

          cv::BFMatcher *bfm = new cv::BFMatcher(cv::NORM_HAMMING, false); // cross-check
//          Mat pdesc_l1= Mat::zeros(Size(32,kf0->lm_descriptor.size()),CV_8U);
//          Mat pdesc_l2= Mat::zeros(Size(32,kf1->lm_descriptor.size()),CV_8U);
          cv::Mat pdesc_l1= cv::Mat::zeros(cv::Size(32,static_cast<int>(kf0->lm_descriptor.size())),CV_8U);
          cv::Mat pdesc_l2= cv::Mat::zeros(cv::Size(32,static_cast<int>(kf1->lm_descriptor.size())),CV_8U);
          vector<vector<cv::DMatch>> pmatches_12, pmatches_21;
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

          vector<cv::Point3f> p3d;
          vector<cv::Point2f> p2d;
          p3d.clear();
          p2d.clear();

          for (size_t i = 0; i < pmatches_12.size(); i++)
          {
              // check if they are mutual best matches
//              int lr_qdx = pmatches_12[i][0].queryIdx;
//              int lr_tdx = pmatches_12[i][0].trainIdx;
//              int rl_tdx = pmatches_21[lr_tdx][0].trainIdx;
             size_t lr_qdx = static_cast<size_t>(pmatches_12[i][0].queryIdx);
             size_t lr_tdx = static_cast<size_t>(pmatches_12[i][0].trainIdx);
             size_t rl_tdx = static_cast<size_t>(pmatches_21[lr_tdx][0].trainIdx);

              // check if they are mutual best matches and satisfy the distance ratio test

              if (lr_qdx == rl_tdx)
              {
                  if(pmatches_12[i][0].distance/pmatches_12[i][1].distance < static_cast<float>(ratioMax))
                  {
                    common_pt++;
                    // save data for optimization
                    //Vector3d P = kf0->lm_3d[lr_qdx];
                    double d = kf0->lm_d[lr_qdx];
                    Vector2d pl_map = kf0->lm_2d[lr_qdx];
                    double x = (pl_map(0)-cx)/fx*d;
                    double y = (pl_map(1)-cy)/fy*d;
                    Vector3d P0(x,y,d);
                    Vector3f P = P0.cast<float>();
                    Vector2f pl_obs = kf1->lm_2d[lr_tdx].cast<float>();
                    cv::Point3f p3(P(0),P(1),P(2));
                    cv::Point2f p2(pl_obs(0),pl_obs(1));
                    p3d.push_back(p3);
                    p2d.push_back(p2);
                  }

              }

          }
          cv::Mat r_ = cv::Mat::zeros(3, 1, CV_64FC1);
          cv::Mat t_ = cv::Mat::zeros(3, 1, CV_64FC1);
          cv::Mat inliers;
          //SE3_to_rvec_tvec(kf0->T_c_w, r_ , t_ );
          //cout<<"start ransac"<<endl;
          if(p3d.size() < 5) return is_lc;
          solvePnPRansac(p3d,p2d,cameraMatrix,distCoeffs,r_,t_,false,100,4.0,0.99,inliers,cv::SOLVEPNP_P3P);
      //    cout<<"selected points size: "<<p3d.size()<<" inliers size: "<<inliers.rows<<" unseletced size: "<<pmatches_12.size()<<endl;

          if(inliers.rows*1.0/p3d.size() < ratioRansac || inliers.rows < minPts ) //return is_lc;
          {
            cout<<"reject loop after geometry check "<<endl;
            cout<<"ratio test: "<<static_cast<double>(inliers.rows)/static_cast<double>(p3d.size())<<" "<<"number test "<<inliers.rows<<endl;
            return is_lc;
          }
          //SE3 se_ij


//          SE3 se_iw = kf0->T_c_w;
//          SE3 se_jw = SE3_from_rvec_tvec(r_,t_);
//          SE3 se_jw_p = kf1->T_c_w;

//          se_ji = se_jw*se_iw.inverse();// i previous kf, j current kf, sji = sjw*swi; from prev to curr
//          //se_ij = se_ji.inverse();
//          SE3 se_j_correct = se_jw*se_jw_p.inverse();

            se_ji = SE3_from_rvec_tvec(r_,t_);

          if(se_ji.translation().norm() < 3 && se_ji.so3().log().norm() < 1.5) is_lc = true;
      }
      return is_lc;
    }


    void expandGraph()
    {
      size_t g_size = kfbv_map.size();
      sim_vec.resize(g_size);

      // sim matrix
      sim_matrix.resize(g_size);
      for (size_t i = 0; i < g_size; i++)
          sim_matrix[i].resize(g_size);

    }


    void loopClosureOnCovGraphG2ONew()
    {
      uint64_t kf_prev_idx = 2 * kf_map_lc.size();
      uint64_t kf_curr_idx = 0;
      for(size_t i = 0; i < loop_ids.size(); i++)
      {
        if(loop_ids[i](0) < static_cast<int>(kf_prev_idx))
          kf_prev_idx = static_cast<uint64_t>(loop_ids[i](0));
        if(loop_ids[i](1) > static_cast<int>(kf_curr_idx))
          kf_curr_idx = static_cast<uint64_t>(loop_ids[i](1));
      }
      cout<<"first and last id in the loop: "<<kf_prev_idx<<" "<<kf_curr_idx<<endl;


      g2o::SparseOptimizer optimizer;
      optimizer.setVerbose(false);


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

      cout<<"start to insert vertices "<<endl;
      vector<int> kf_list;
      for (size_t i = kf_prev_idx; i <= kf_curr_idx; i++)
      {
          if (kf_map_lc[i] != nullptr)
          {
              // check if it is a LC vertex
              //bool is_lc_i = false;
              bool is_lc_j = false;
              int id = 0;
              for (auto it = loop_ids.begin(); it != loop_ids.end(); it++, id++)
              {
                  if ((*it)(0) == static_cast<int>(i))
                  {
                      //is_lc_i = true;
                      break;
                  }
                  if ((*it)(1) == static_cast<int>(i))
                  {
                      is_lc_j = true;
                      break;
                  }
              }
              kf_list.push_back(static_cast<int>(i));
              // create SE3 vertex
              g2o::VertexSE3 *v_se3 = new g2o::VertexSE3();
              v_se3->setId(static_cast<int>(i));
              v_se3->setMarginalized(false);

              // _inverseMeasurement * from->estimate().inverse() * to->estimate();
              // z^-1 * (x_i^-1 * x_j)
              SE3 siw = kf_map_lc[i]->T_c_w.inverse();
             // cout<<siw.so3()<<" "<<siw.translation()<<endl;

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
                  if(i == 0 || i == kf_prev_idx)
                      v_se3->setFixed(true);
                  else
                      v_se3->setFixed(false);
              }
              optimizer.addVertex(v_se3);
          }
      }
      cout<<"start to insert adjacent edge "<<endl;


      // introduce edges
      for (size_t i = kf_prev_idx; i <= kf_curr_idx; i++)
      {

          for (size_t j = i + 1; j <= min(kf_curr_idx,i + 5);  j++)//, j <= i +5
          {
              if (kf_map_lc[i] != nullptr && kf_map_lc[j] != nullptr)
              {
                  // kf2kf constraint
                  SE3 sji = kf_map_lc[j]->T_c_w*kf_map_lc[i]->T_c_w.inverse();//se_jw*se_iw.inverse();
                  SE3 sij = sji.inverse();
                  //cout<<sji.so3()<<" "<<sji.translation()<<endl;

                  // add edge
                  g2o::EdgeSE3* e_se3 = new g2o::EdgeSE3();

                  e_se3->setVertex(0, optimizer.vertex(static_cast<int>(i)));
                  e_se3->setVertex(1, optimizer.vertex(static_cast<int>(j)));
                  e_se3->setMeasurement(SE3_to_g2o(sij));
                  //e_se3->information() = map_keyframes[j]->xcov_kf_w;
                  e_se3->setInformation(Matrix6d::Identity());
                  e_se3->setRobustKernel(new g2o::RobustKernelCauchy());
                                         //RobustKernelHuber());
                  optimizer.addEdge(e_se3);
              }
          }
      }
      cout<<"start to insert loop edge "<<endl;

      // introduce loop closure edges
      uint64_t id = 0;
      for (auto it = loop_ids.begin(); it != loop_ids.end(); it++, id++)
      {
          // add edge
          g2o::EdgeSE3 *e_se3 = new g2o::EdgeSE3();
          e_se3->setVertex(0, optimizer.vertex((*it)(0)));
          e_se3->setVertex(1, optimizer.vertex((*it)(1)));
          SE3 loop_pose = loop_poses[id].inverse();
          e_se3->setMeasurement(SE3_to_g2o(loop_pose));
          e_se3->information() = Matrix6d::Identity();
          e_se3->setRobustKernel(new g2o::RobustKernelCauchy());
          optimizer.addEdge(e_se3);
      }

      //optimizer.edges();

      cout<<"start to optimize "<<endl;
      // optimize graph
      optimizer.initializeOptimization();
      optimizer.computeInitialGuess();
      optimizer.computeActiveErrors();
      optimizer.optimize(100);

      // recover pose and update map

      for (auto kf_it = kf_list.begin(); kf_it != kf_list.end(); kf_it++)
      {
          g2o::VertexSE3 *v_se3 = static_cast<g2o::VertexSE3 *>(optimizer.vertex((*kf_it)));
          g2o::SE3Quat Twc_g2o = v_se3->estimateAsSE3Quat();
          SE3 Tcw1 = kf_map_lc[static_cast<size_t>(*kf_it)]->T_c_w;

          SE3 Tw2c = SE3_from_g2o(Twc_g2o);
          SE3 Tw2_w1 =Tw2c*Tcw1;// transorm from previous to current from odom to map
          SE3 Tw1_w2 = Tw2_w1.inverse();

          kf_map_lc[static_cast<size_t>(*kf_it)]->T_c_w = Tw2c.inverse();
          path_lc_pub->pubPathT_w_c(Tw2c,ros::Time::now());
          //cout<<"after g2o pgo: ";
          //cout<<Tcw_corr<<endl;
          SE3 odom = kf_map_lc[static_cast<size_t>(*kf_it)]->T_c_w_odom;
          if(kf_it == kf_list.end()-1)
          {

            T_prevmap_map = Tw1_w2;
           // cout<<"from w2 to w1: "<<Tw1_w2.so3()<<" "<<Tw1_w2.translation()<<endl;
            T_odom_map = T_odom_map*Tw1_w2;
           // cout<<"from w2 to w0: "<<T_odom_map.so3()<<" "<<T_odom_map.translation()<<endl;
            vmap_correct.push_back(Tw1_w2);
          //  cout<<"did match initial ? "<<Tw2c.inverse().so3()<<" "<<Tw2c.inverse().translation()<<endl;
           // cout<<"did match initial ? "<<odom.so3()<<" "<<odom.translation()<<endl;
          }

      }
      

      


    }


    void frame_callback(const flvis::KeyFrameConstPtr& msg)
    {
      

      
        tic_toc_ros unpack_tt;
        sim_vec.clear();
        KeyFrameLC kf;
        BowVector kf_bv;
        SE3 loop_pose;

        cv::Mat img_unpack, d_img_unpack;
        vector<int64_t> lm_id_unpack;
        vector<Vec3> lm_3d_unpack;
        vector<Vec2> lm_2d_unpack;
        vector<cv::Mat>     lm_descriptor_unpack;
        int lm_count_unpack;
        
        
        KeyFrameMsg::unpack(msg,kf.frame_id,img_unpack,d_img_unpack,lm_count_unpack,
                            lm_id_unpack,lm_2d_unpack,lm_3d_unpack,lm_descriptor_unpack,kf.T_c_w_odom,kf.t);
        if(kf.frame_id < 40)
          return;

        kf.T_c_w = kf.T_c_w_odom*T_odom_map;
        kf.keyframe_id = kf_id++;

        path_lc_pub->pubPathT_c_w(kf.T_c_w,kf.t);



        //cout<<"unpack cost: ";
        unpack_tt.toc();

        cout<<"send transform between map and odom: "<<endl;

        SE3 T_map_odom = T_odom_map.inverse();

        Quaterniond q_tf = T_map_odom.so3().unit_quaternion();
        Vec3        t_tf = T_map_odom.translation();


        transform.setOrigin(tf::Vector3(t_tf[0],t_tf[1],t_tf[2]));
        transform.setRotation(tf::Quaternion(q_tf.x(),q_tf.y(),q_tf.z(),q_tf.w()));

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

        tic_toc_ros feature_tt;


        vector<cv::KeyPoint> ORBFeatures;
        vector<cv::Point2f>  kps;
        cv::Mat ORBDescriptorsL;
        vector<cv::Mat> ORBDescriptors;

        kps.clear();
        ORBFeatures.clear();
        ORBDescriptors.clear();

        cv::Ptr<cv::ORB> orb = cv::ORB::create(500,1.2f,8,31,0,2, cv::ORB::HARRIS_SCORE,31,20);
        orb->detectAndCompute(img_unpack,cv::Mat(),ORBFeatures,ORBDescriptorsL);

        cv::KeyPoint::convert(ORBFeatures,kps);
        descriptors_to_vMat(ORBDescriptorsL,ORBDescriptors);



        kf.lm_descriptor = ORBDescriptors;

       //cout<<"descriptor numbers: "<<ORBDescriptors.size()<<endl;
       // cout<<"feature cost: ";feature_tt.toc();


        //pass feature and descriptor
        vector<Vec2> lm_2d;
        vector<double> lm_d;
        for(size_t i = 0; i<ORBFeatures.size();i++)
        {
          cv::Point2f cvtmp = ORBFeatures[i].pt;
          Vec2 tmp(cvtmp.x,cvtmp.y);
          double d = (d_img_unpack.at<ushort>(cvtmp))/1000;
          lm_2d.push_back(tmp);
          lm_d.push_back(d);
        }
        kf.lm_2d = lm_2d;
        kf.lm_d = lm_d;
        kf.lm_count = static_cast<int>(lm_2d.size());
       // cout<<"pass feature number: "<<kf.lm_count;
        lm_2d.clear();
        lm_d.clear();




        tic_toc_ros bow_tt;

        voc.transform(kf.lm_descriptor,kf_bv);
        kf.kf_bv = kf_bv;
       // cout<<"bow transfer cost: ";bow_tt.toc();

        shared_ptr<KeyFrameLC> kf_lc_ptr =std::make_shared<KeyFrameLC>(kf);
        kf_map_lc.push_back(kf_lc_ptr);     
        kfbv_map.push_back(kf_bv);
        expandGraph();




        tic_toc_ros bow_find_tt;

        for (uint64_t i = 0; i < kf_map_lc.size(); i++)
        {
          if(kf_map_lc[i] != nullptr)
          {
            double score = voc.score(kf_bv,kf_map_lc[i]->kf_bv);
            sim_matrix[kf_map_lc.size()-1][i] = score;
            sim_matrix[i][kf_map_lc.size()-1] = score;
            sim_vec[i] = score;
          }
          else {
            sim_matrix[kf_map_lc.size()-1][i] = 0;
            sim_matrix[i][kf_map_lc.size()-1] = 0;
            sim_vec[i] = 0;

          }
        }
        //cout<<"bow find cost: ";
        bow_find_tt.toc();

        if(kf_id < 50)
        {
          //cout<<"KF number is less than 50. Return."<<endl;
          return;
        }



        uint64_t kf_prev_idx;
        bool is_lc_candidate = isLoopCandidate(kf_prev_idx);


        if(!is_lc_candidate)
        {
          cout<<"no loop candidate."<<endl;
          return;
        }
        else
        {
          cout<<"has loop candidate."<<endl;
        }


        bool is_lc = false;
        uint64_t kf_curr_idx = kf_map_lc.size()-1;



        is_lc = isLoopClosureKF(kf_map_lc[kf_prev_idx], kf_map_lc[kf_curr_idx], loop_pose);
        if(!is_lc)
        {
          cout<<"Geometry test fails."<<endl;
          return;

        }
        else {
          cout<<"Pass geometry test."<<endl;
        }
        if(is_lc)
        {

          loop_ids.push_back(Vec3I(static_cast<int>(kf_prev_idx), static_cast<int>(kf_curr_idx), 1));
          loop_poses.push_back(loop_pose);

          int thre = static_cast<int>((static_cast<double>(kf_id)/100)*2);

          if(kf_curr_idx - static_cast<size_t>(last_pgo_id) < thre)
            cout<<"Last loop is too close."<<endl;

          if(kf_curr_idx - static_cast<size_t>(last_pgo_id) > thre)
          {
            path_lc_pub->clearPath();
            tic_toc_ros pgo;
            loopClosureOnCovGraphG2ONew();
            last_pgo_id = static_cast<int>(kf_curr_idx);
            cout<<"pose graph takes: "<<endl;
            pgo.toc();
          }
        }

        kf_lc_ptr->T_c_w = kf_lc_ptr->T_c_w_odom*T_odom_map;
        T_map_odom = T_odom_map.inverse();

        q_tf = T_map_odom.so3().unit_quaternion();
        t_tf = T_map_odom.translation();


        transform.setOrigin(tf::Vector3(t_tf[0],t_tf[1],t_tf[2]));
        transform.setRotation(tf::Quaternion(q_tf.x(),q_tf.y(),q_tf.z(),q_tf.w()));

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
    }

    virtual void onInit()
    {

        ros::NodeHandle nh = getNodeHandle();

        string configFilePath;
        nh.getParam("/yamlconfigfile",   configFilePath);
        int cam_type_from_yaml = getIntVariableFromYaml(configFilePath,"type_of_cam");
        if(cam_type_from_yaml==0) cam_type=DEPTH_D435I;
        if(cam_type==DEPTH_D435I)
        {
            cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath,"cam0_intrinsics");
            distCoeffs   = distCoeffsFromYaml(configFilePath,"cam0_distortion_coeffs");
            fx = cameraMatrix.at<double>(0,0);
            fy = cameraMatrix.at<double>(1,1);
            cx = cameraMatrix.at<double>(0,2);
            cy = cameraMatrix.at<double>(1,2);
        }
        image_width  = getIntVariableFromYaml(configFilePath,"image_width");
        image_height = getIntVariableFromYaml(configFilePath,"image_height");
//        cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath);
//        distCoeffs = distCoeffsFromYaml(configFilePath);
//        fx = cameraMatrix.at<double>(0,0);
//        fy = cameraMatrix.at<double>(1,1);
//        cx = cameraMatrix.at<double>(0,2);
//        cy = cameraMatrix.at<double>(1,2);
        cout << "cameraMatrix:" << endl << cameraMatrix << endl
             << "distCoeffs:" << endl << distCoeffs << endl
             << "image_width: "  << image_width << " image_height: "  << image_height << endl
             << "fx: "  << fx << " fy: "  << fy <<  " cx: "  << cx <<  " cy: "  << cy << endl;
        string vocFile;


        nh.getParam("/voc", vocFile);
        Vocabulary vocTmp(vocFile);
        cout<<"voc begin: "<<endl;
        voc = vocTmp;
        kf_id = 0;
        Database dbTmp(voc, true, 0);
        db = dbTmp;

        path_lc_pub  = new RVIZPath(nh,"/vision_path_lc_all","map");

        sub_kf = nh.subscribe<flvis::KeyFrame>(
                    "/vo_kf",
                    10,
                    boost::bind(&LoopClosingNodeletClass::frame_callback, this, _1));

    }




};//class LoopClosingNodeletClass
}//namespace flvis_ns



PLUGINLIB_EXPORT_CLASS(flvis_ns::LoopClosingNodeletClass, nodelet::Nodelet)


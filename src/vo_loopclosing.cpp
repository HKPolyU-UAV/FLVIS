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
//#include <g2o/types/sba/types_six_dof_expmap.h>

#include <utils/tic_toc_ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <condition_variable>
#include <octomap/octomap.h>
#include <octomap/OccupancyOcTreeBase.h>

typedef pcl::PointXYZ PointP;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZI PointI;

typedef pcl::PointCloud<PointP> PointCloudP;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointI> PointCloudI;


using namespace DBoW3;
using namespace cv;
using namespace std;


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

#define lcKFStart (25)
#define lcKFDist (18)
#define lcKFMaxDist (50)
#define lcKFLast (20)
#define lcNKFClosest (3)
#define ratioMax (0.7)
#define ratioRansac (0.5)
#define minPts (35)

struct KeyFrameLC
{
  Mat d_img;
  int64_t         frame_id;
  int64_t         keyframe_id;
  int             lm_count;
  vector<Vec2>    lm_2d;
  vector<double>  lm_d;
  vector<Mat>     lm_descriptor;
  BowVector       kf_bv;
  SE3             T_c_w_odom;
  SE3             T_c_w;
  ros::Time       t;
};




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

    //kf id
    int64_t kf_id = 0;
    //last loop id
    int64_t last_pgo_id = -1000;

    //mapping
    PointCloudP::Ptr globalPC = boost::make_shared<PointCloudP>();
    PointCloudP::Ptr globalafterPC = boost::make_shared<PointCloudP>();
    PointCloudP::Ptr globalvoPC = boost::make_shared<PointCloudP>();
    shared_ptr<octomap::OcTree> globalOctreePtr = make_shared<octomap::OcTree>(0.4);
    //octomap::OcTree globalOctree = octomap::OcTree(0.05);
    pcl::PointCloud<PointP>::Ptr generatePointCloudP(Mat& depth)
    {
      PointCloudP::Ptr tmp(new PointCloudP());
      for(int m =0; m<depth.rows; m+=5)
      {
        for(int n = 0; n<depth.cols; n+=5)
        {
          float d = depth.ptr<ushort>(m)[n]/1000.0;
          if(d<0.01 || d>4)
            continue;
          pcl::PointXYZ p;
          p.z = d;
          p.x = (n-cx)*d/fx;
          p.y = (m-cy)*d/fy;

          tmp->points.push_back(p);

        }
      }
      tmp->is_dense = false;
      return tmp;
    }


    pcl::PointCloud<PointP>::Ptr transformPointCloudP(SE3 pose, pcl::PointCloud<PointP>& pointCloudP)
    {
      PointCloudP::Ptr cloud(new PointCloudP);
      Matrix<double,4,4> transformMat = pose.matrix().inverse();
      Matrix<float,4,4> transformMatD = transformMat.cast<float>();
      pcl::transformPointCloud(pointCloudP, *cloud, transformMatD);// transform from cam to world
      cloud->is_dense = false;
      return cloud;
    }
    octomap::Pointcloud generateOctomapPC(Mat& depth)
    {

      octomap::Pointcloud tmp;
      for(int m =0; m<depth.rows; m+=5)
      {
        for(int n = 0; n<depth.cols; n+=5)
        {
          float d = depth.ptr<float>(m)[n]/1000.0;
          if(d<0.01 || d>4)
            continue;

          double z = d;
          double x = (n-cx)*d/fx;
          double y = (m-cy)*d/fy;

          octomap::point3d p(x,y,z);

          tmp.push_back(p);

        }
      }

    }

    octomap::Pointcloud transformOctomapPC(SE3 pose, octomap::Pointcloud octoP)
    {
      cout<<"start pose trans:"<<endl;
      Vector3d trans = pose.translation();
      Quaterniond rots = pose.unit_quaternion();
     cout<<"start double to float"<<endl;
      float x = trans(0);
      float y = trans(1);
      float z = trans(2);
      float qw = rots.w();
      float qx = rots.x();
      float qy = rots.y();
      float qz = rots.z();

      cout<<"double to float"<<endl;

      octomath::Vector3 tran(x,y,z);
      octomath::Quaternion rot(qw,qx,qy,qz);


      octomap::pose6d poseOcto(tran, rot);

      octomap::Pointcloud tmp(octoP);
      tmp.transform(poseOcto.inv());

      //origin = poseOcto.inv().trans();

      return tmp;

    }






    bool isLoopCandidate( uint64_t &kf_prev_idx)
    {
      bool is_lc_candidate = false;
      int g_size = kf_map_lc.size();
      //cout<<"kf size: "<<g_size<<endl;
      vector<Vector2d> max_sim_mat;
      if(g_size < 40) return is_lc_candidate;
      for (uint64_t i = 0; i < (uint64_t)(g_size - lcKFDist); i++)
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
      for (uint64_t i = (uint64_t)(g_size - lcKFDist); i < (uint64_t)(g_size); i++)
      {
          double score_i = sim_matrix[i][g_size-1];
          if (score_i < lc_min_score && score_i > 0.001) lc_min_score = score_i;
      }

      lc_min_score = min(lc_min_score, 0.4);
      if( max_sim_mat[0](1) < max(0.2, lc_min_score)) return is_lc_candidate;

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
       if (nkf_closest >= lcNKFClosest && max_sim_mat[0](1) > 0.2 )
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
          if(p3d.size() < 5) return is_lc;
          solvePnPRansac(p3d,p2d,cameraMatrix,distCoeffs,r_,t_,false,100,3.0,0.99,inliers,SOLVEPNP_P3P);
          cout<<"selected points size: "<<p3d.size()<<" inliers size: "<<inliers.rows<<" unseletced size: "<<pmatches_12.size()<<endl;
          cout<<"ratio test: "<<inliers.rows*1.0/p3d.size()<<" "<<"number test "<<inliers.rows<<endl;

          if(inliers.rows*1.0/p3d.size() < ratioRansac || inliers.rows < minPts ) //return is_lc;
          {
            cout<<"dont believe: "<<endl;
            cout<<"ratio test: "<<(double)inliers.rows/(double)p3d.size()<<" "<<"number test "<<inliers.rows<<endl;
            return is_lc;
          }

          cout<<"after ransac test: "<<endl;

          SE3 se_iw = kf0->T_c_w;
          SE3 se_jw = SE3_from_rvec_tvec(r_,t_);
          SE3 se_jw_p = kf1->T_c_w;

          se_ji = se_jw*se_iw.inverse();// i previous kf, j current kf, sji = sjw*swi; from prev to curr
          //se_ij = se_ji.inverse();
          SE3 se_j_correct = se_jw*se_jw_p.inverse();

          if(se_ji.translation().norm() < 1 && se_ji.so3().log().norm() < 0.8) is_lc = true;
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
                    //Vector3d P = kf0->lm_3d[lr_qdx];
                    double d = kf0->lm_d[lr_qdx];
                    Vector2d pl_map = kf0->lm_2d[lr_qdx];
                    double x = (pl_map(0)-cx)/fx*d;
                    double y = (pl_map(1)-cy)/fy*d;
                    Vector3d P(x,y,d);
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
          //SE3_to_rvec_tvec(kf0->T_c_w, r_ , t_ );
          cout<<"start ransac"<<endl;
          if(p3d.size() < 5) return is_lc;
          solvePnPRansac(p3d,p2d,cameraMatrix,distCoeffs,r_,t_,false,100,3.0,0.99,inliers,SOLVEPNP_P3P);
          cout<<"selected points size: "<<p3d.size()<<" inliers size: "<<inliers.rows<<" unseletced size: "<<pmatches_12.size()<<endl;

          if(inliers.rows*1.0/p3d.size() < ratioRansac || inliers.rows < minPts ) //return is_lc;
          {
            cout<<"reject loop after geometry check "<<endl;
            cout<<"ratio test: "<<(double)inliers.rows/(double)p3d.size()<<" "<<"number test "<<inliers.rows<<endl;
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

          if(se_ji.translation().norm() < 1 && se_ji.so3().log().norm() < 0.8) is_lc = true;
      }
      return is_lc;
    }


    void expandGraph()
    {
      int g_size = (int) kfbv_map.size();
      sim_vec.resize(g_size);

      // sim matrix
      sim_matrix.resize(g_size);
      for (int i = 0; i < g_size; i++)
          sim_matrix[i].resize(g_size);

    }


    void loopClosureOnCovGraphG2ONew()
    {
      uint64_t kf_prev_idx = 2 * kf_map_lc.size();
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
      for (int i = kf_prev_idx; i <= kf_curr_idx; i++)
      {
          if (kf_map_lc[i] != nullptr)
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
      for (int i = kf_prev_idx; i <= kf_curr_idx; i++)
      {

          for (int j = i + 1; j <= min((int)kf_curr_idx,i +5);  j++)//, j <= i +5
          {
              if (kf_map_lc[i] != nullptr && kf_map_lc[j] != nullptr)
              {
                  // kf2kf constraint
                  SE3 sji = kf_map_lc[j]->T_c_w*kf_map_lc[i]->T_c_w.inverse();//se_jw*se_iw.inverse();
                  SE3 sij = sji.inverse();
                  //cout<<sji.so3()<<" "<<sji.translation()<<endl;

                  // add edge
                  g2o::EdgeSE3* e_se3 = new g2o::EdgeSE3();

                  e_se3->setVertex(0, optimizer.vertex(i));
                  e_se3->setVertex(1, optimizer.vertex(j));
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
          SE3 Tcw1 = kf_map_lc[(*kf_it)]->T_c_w;
          SE3 Tw2c = SE3_from_g2o(Twc_g2o);
          SE3 Tw2_w1 =Tw2c*Tcw1;// transorm from previous to current from odom to map
          SE3 Tw1_w2 = Tw2_w1.inverse();

          kf_map_lc[(*kf_it)]->T_c_w = Tw2c.inverse();
          //cout<<"after g2o pgo: ";
          //cout<<Tcw_corr<<endl;
          SE3 odom = kf_map_lc[(*kf_it)]->T_c_w_odom;
          if(kf_it == kf_list.end()-1)
          {

            T_prevmap_map = Tw1_w2;
            cout<<"from w2 to w1: "<<Tw1_w2.so3()<<" "<<Tw1_w2.translation()<<endl;
            T_odom_map = T_odom_map*Tw1_w2;
            cout<<"from w2 to w0: "<<T_odom_map.so3()<<" "<<T_odom_map.translation()<<endl;
            vmap_correct.push_back(Tw1_w2);
            cout<<"did match initial ? "<<Tw2c.inverse().so3()<<" "<<Tw2c.inverse().translation()<<endl;
            cout<<"did match initial ? "<<odom.so3()<<" "<<odom.translation()<<endl;
          }

      }



    }


    void frame_callback(const vo_nodelet::KeyFrameConstPtr& msg)
    {
        sim_vec.clear();
        KeyFrameStruct kf;
        BowVector kf_bv;
        SE3 loop_pose;

        KeyFrameMsg::unpack(msg,kf.frame_id,kf.img,kf.d_img,kf.lm_count,kf.lm_id,kf.lm_2d,kf.lm_3d,kf.lm_descriptor,kf.T_c_w);
        //cout<<"unpack ok: "<<endl;
        shared_ptr<KeyFrameStruct> kf_ptr = make_shared<KeyFrameStruct>(kf);
        // new feature detector and descriptor
        tic_toc_ros tt_kpdes;
        Ptr<FastFeatureDetector> detector= FastFeatureDetector::create();
        vector<KeyPoint> FASTFeatures;
        vector<Point2f>  kps;
        detector->detect(kf.img, FASTFeatures);
        KeyPoint::convert(FASTFeatures,kps);
        //cout<<"fast: "<<FASTFeatures.size()<<"kps: "<<kps.size()<<endl;
        Mat tmpDescriptors;
        vector<Mat> newDescriptors;
        Ptr<DescriptorExtractor> extractor = ORB::create();
        extractor->compute(kf.img, FASTFeatures, tmpDescriptors);
        descriptors_to_vMat(tmpDescriptors,newDescriptors);
        voc.transform(newDescriptors,kf_bv);
        //voc.transform(kf.lm_descriptor,kf_bv);
        //cout<<"des size: "<<newDescriptors.size()<<"kp size: "<<kps.size()<<"fast size: "<<FASTFeatures.size()<<endl;

        //cout<<"tmpDes size: "<<tmpDescriptors.rows<<" "<<tmpDescriptors.cols<<endl;




        //save to new kf vector
        shared_ptr<KeyFrameLC> kf_lc_ptr =std::make_shared<KeyFrameLC>();
        kf_lc_ptr->t = ros::Time::now();

        kf_lc_ptr->frame_id = kf_ptr->frame_id;
        kf_lc_ptr->d_img = kf_ptr->d_img;
        kf_lc_ptr->keyframe_id = kf_id;
        vector<Vec2> lm_2d;
        vector<double> lm_d;
        for(size_t i = 0; i<FASTFeatures.size();i++)
        {
          Point2f cvtmp = FASTFeatures[i].pt;
          Vec2 tmp(cvtmp.x,cvtmp.y);
          double d = (kf_lc_ptr->d_img.at<ushort>(cvtmp))/1000;
          lm_2d.push_back(tmp);
          lm_d.push_back(d);
        }
        kf_lc_ptr->lm_2d = lm_2d;
        kf_lc_ptr->lm_d = lm_d;
        kf_lc_ptr->lm_count = lm_2d.size();
        lm_2d.clear();
        lm_d.clear();
        kf_lc_ptr->lm_descriptor = newDescriptors;
        kf_lc_ptr->kf_bv = kf_bv;
        kf_lc_ptr->T_c_w_odom = kf_ptr->T_c_w;
        kf_lc_ptr->T_c_w = kf_ptr->T_c_w*T_odom_map;// T_c_w c ccurrent camera, w odom,
        kf_id +=1;
        FASTFeatures.clear();
        kps.clear();
        newDescriptors.clear();
        //voc.transform(kf_ptr->lm_descriptor,kf_bv);
        //kf_map.push_back(kf_ptr);
        kf_map_lc.push_back(kf_lc_ptr);
        kfbv_map.push_back(kf_bv);
        expandGraph();

        tic_toc_ros pc_insertion;
        pcl::PointCloud<PointP>::Ptr tmpPtr = generatePointCloudP(kf_lc_ptr->d_img);
        pcl::PointCloud<PointP>::Ptr tmpPtrW = transformPointCloudP(kf_lc_ptr->T_c_w, *tmpPtr);
        *globalPC += *tmpPtrW;
        cout<<"point cloud transfrom and insertion";
        pc_insertion.toc();
        pcl::io::savePLYFileBinary("/home/lsgi/out/map.ply", *globalPC);
       // pcl::io::savePCDFileASCII("/home/lsgi/out/globalPC.pcd", *globalPC);

        //octomap::point3d origin(0.0,0.0,0.0);
        //cout<<"generate octo pc"<<endl;
        tic_toc_ros octo_insertion;
        shared_ptr<octomap::Pointcloud> octomapTmpCloud = make_shared<octomap::Pointcloud>();//(new octomap::Pointcloud());
        octomapTmpCloud->reserve(tmpPtrW->size());
        tic_toc_ros get_octo_points;
        for (PointCloudP::const_iterator it = tmpPtrW->begin(); it != tmpPtrW->end(); ++it){
          if (!std::isnan(it->z)) octomapTmpCloud->push_back(it->x, it->y, it->z);
        }
        get_octo_points.toc();
        Vector3d trans = kf_lc_ptr->T_c_w.translation();
        Quaterniond rots = kf_lc_ptr->T_c_w.unit_quaternion();
        float x = trans(0);
        float y = trans(1);
        float z = trans(2);
        float qw = rots.w();
        float qx = rots.x();
        float qy = rots.y();
        float qz = rots.z();
        octomath::Vector3 tran(x,y,z);
        octomath::Quaternion rot(qw,qx,qy,qz);


        octomap::pose6d poseOcto(tran, rot);
        octomap::point3d origin = poseOcto.inv().trans();

        tic_toc_ros octo_in;
        globalOctreePtr->insertPointCloud(*octomapTmpCloud, origin);
        octo_in.toc();
        tic_toc_ros octo_update;
        globalOctreePtr->updateInnerOccupancy();
        octo_update.toc();

        cout<<"point cloud transfrom and insertion";
        octo_insertion.toc();
        //cout<<"write octo pc "<<endl;
        globalOctreePtr->writeBinary("/home/lsgi/out/globalOcree.bt");




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
        uint64_t kf_prev_idx;
        bool is_lc_candidate = isLoopCandidate(kf_prev_idx);
        //cout<<"loop frames: "<<kf_prev_idx<<" "<<kf_map_lc.size()-1<<endl;

        bool is_lc = false;
        uint64_t kf_curr_idx = kf_map_lc.size()-1;

//        if(kf_curr_idx > lcKFStart) bool is_lc = isLoopClosure(kf_map[kf_curr_idx-5],kf_map[kf_curr_idx],loop_pose);
//        {
//          tt_le.toc();
//          tic_toc_ros tt_pgo;
//          loop_ids.push_back(Vec3I(kf_curr_idx-5, kf_curr_idx, 1));
//          loop_poses.push_back(loop_pose);
//          loopClosureOnCovGraphG2O();
//          tt_pgo.toc();
//        }

        // real loop clpsure correction, but need to tune paras
        if(!is_lc_candidate) return;

        cout<<kf_map_lc[kf_curr_idx]->lm_descriptor.size()<<endl;
        if(kf_curr_idx > lcKFStart) is_lc = isLoopClosureKF(kf_map_lc[kf_prev_idx], kf_map_lc[kf_curr_idx], loop_pose);
        if(!is_lc) return;
        if(is_lc)
        {
          loop_ids.push_back(Vec3I(kf_prev_idx, kf_curr_idx, 1));
          loop_poses.push_back(loop_pose);
          if(kf_curr_idx - last_pgo_id > 3)
          {
            tic_toc_ros pgo;
            loopClosureOnCovGraphG2ONew();
            last_pgo_id = kf_curr_idx;
            pgo.toc();
          }
        }

        if(kf_curr_idx == 200)
        {

          for(size_t i = 0; i<kf_map_lc.size();i++)
          {
            pcl::PointCloud<PointP>::Ptr tmpPC1 = generatePointCloudP(kf_map_lc[i]->d_img);
            pcl::PointCloud<PointP>::Ptr tmpPCinMap = transformPointCloudP(kf_map_lc[i]->T_c_w, *tmpPC1);
            pcl::PointCloud<PointP>::Ptr tmpPCinOdom = transformPointCloudP(kf_map_lc[i]->T_c_w_odom, *tmpPC1);
            *globalafterPC += *tmpPCinMap;
            *globalvoPC += *tmpPCinOdom;
          }
          pcl::io::savePLYFileBinary("/home/lsgi/out/aftermap.ply", *globalafterPC);
          pcl::io::savePLYFileBinary("/home/lsgi/out/vo.ply", *globalvoPC);
        }







        //imshow("loopclosing", img);
        //waitKey(1);

        std::ofstream of;
        of.open("/home/lsgi/out/sim_mat.txt");

        for(size_t i = 0; i < sim_matrix.size(); i++){
          for(size_t j = 0;j < sim_matrix.size(); j++){
            of<<sim_matrix[i][j]<<" ";
          }
          of << "\n";
        }

        of.close();



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
        cout<<"voc begin: "<<endl;
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


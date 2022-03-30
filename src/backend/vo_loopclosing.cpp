//STDLIB
#include <iostream>
#include <fstream>
#include <deque>
#include <stdint.h>
#include <condition_variable>

//ROS
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <image_transport/image_transport.h>

//FLVIS
#include <include/yamlRead.h>
#include <include/triangulation.h>
#include <include/keyframe_msg.h>
#include <flvis/KeyFrame.h>
#include <include/camera_frame.h>
#include <include/vi_type.h>
#include <include/tic_toc_ros.h>
#include <include/rviz_path.h>

//OPENCV
#include <opencv2/opencv.hpp>

//DBoW3
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

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


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

struct LC_PARAS
{
    int lcKFStart;
    int lcKFDist;
    int lcKFMaxDist;
    int lcKFLast;
    int lcNKFClosest;
    double ratioMax;
    double ratioRansac;
    int minPts;
    double minScore;
};

struct KeyFrameLC
{
    int64_t         frame_id;
    int64_t         keyframe_id;
    int             lm_count;
    ros::Time       t;
    SE3             T_c_w;
    SE3             T_c_w_odom;
    vector<Vec2>    lm_2d;
    vector<Vec3>    lm_3d;
    vector<double>  lm_depth;
    vector<cv::Mat> lm_descriptor;
    BowVector       kf_bv;
};

class LoopClosingNodeletClass : public nodelet::Nodelet
{
public:
    LoopClosingNodeletClass()  {;}
    ~LoopClosingNodeletClass() {;}

private:
    ros::Subscriber sub_kf;
    DepthCamera dc;

    ros::Time tt;
    tf::Transform transform;
    tf::TransformBroadcaster br;

    LC_PARAS lc_paras;
    //DBow related para
    Vocabulary voc;
    Database db;// faster search
    vector<vector<double>> sim_matrix;//for similarity visualization
    vector<double> sim_vec;
    //KF database
    //vector<shared_ptr<KeyFrameStruct>> kf_map;
    vector<BowVector> kfbv_map;
    vector<shared_ptr<KeyFrameLC>> kf_map_lc;
    vector<cv::Mat> kf_img_lc;
    vector<cv::DMatch> select_match;
    image_transport::Publisher loop_closure_pub;
    uint64_t kf_curr_idx_;
    uint64_t kf_prev_idx_;


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
    int64_t last_pgo_id = -5000;

    bool isLoopCandidate( uint64_t &kf_prev_idx)
    {
        //cout<<"start to find loop candidate."<<endl;
        bool is_lc_candidate = false;
        size_t g_size = kf_map_lc.size();
        //cout<<"kf size: "<<g_size<<endl;
        vector<Vector2d> max_sim_mat;
        if(g_size < 40) return is_lc_candidate;
        uint64_t sort_index;
        if((g_size - lc_paras.lcKFDist) > 5000)
            sort_index = g_size - lc_paras.lcKFDist -5000;
        else
            sort_index = 0;
        for (uint64_t i = sort_index; i < static_cast<uint64_t>(g_size - lc_paras.lcKFDist); i++)
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
        for (uint64_t i = static_cast<uint64_t>(g_size - lc_paras.lcKFDist); i < static_cast<uint64_t>(g_size); i++)
        {
            double score_i = sim_matrix[i][g_size-1];
            if (score_i < lc_min_score && score_i > 0.001) lc_min_score = score_i;
        }
        //cout << "lc_min_score is " << lc_min_score << endl;
        lc_min_score = min(lc_min_score, 0.4);
        ///cout<< "max sim score is: "<< max_sim_mat[0](1)<<endl;
        if( max_sim_mat[0](1) < max(lc_paras.minScore, lc_min_score)) return is_lc_candidate;

        int idx_max = int(max_sim_mat[0](0));
        int nkf_closest = 0;
        if (max_sim_mat[0](1) >= lc_min_score)
        {
            // there must be at least lc_nkf_closest KFs conected to the LC candidate with a score above lc_dbow_score_min
            for (uint64_t i = 1; i < max_sim_mat.size(); i++)
            {
                int idx = int(max_sim_mat[i](0));

                        if (abs(idx - idx_max) <= lc_paras.lcKFMaxDist && max_sim_mat[i](1) >= lc_min_score * 0.8) nkf_closest++;
            }
        }


        // update in case of being loop closure candidate
        if (nkf_closest >= lc_paras.lcNKFClosest && max_sim_mat[0](1) > lc_paras.minScore)
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

    bool isLoopClosureKF(shared_ptr<KeyFrameLC> kf0, shared_ptr<KeyFrameLC> kf1,SE3 &se_ji)
    {
        //kf0 previous kf, kf1 current kf,
        bool is_lc = false;
        int common_pt = 0;

        vector<cv::DMatch> selected_matches_;

        if (!(kf1->lm_descriptor.size() == 0) && !(kf0->lm_descriptor.size() == 0))
        {


            cv::BFMatcher *bfm = new cv::BFMatcher(cv::NORM_HAMMING, false); // cross-check

            cv::Mat pdesc_l1= cv::Mat::zeros(cv::Size(32,static_cast<int>(kf0->lm_descriptor.size())),CV_8U);
            cv::Mat pdesc_l2= cv::Mat::zeros(cv::Size(32,static_cast<int>(kf1->lm_descriptor.size())),CV_8U);
            vector<vector<cv::DMatch>> pmatches_12, pmatches_21;
            // 12 and 21 matches
            vecDesciptor_to_descriptors(kf0->lm_descriptor,pdesc_l1);
            vecDesciptor_to_descriptors(kf1->lm_descriptor,pdesc_l2);

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
                    if(pmatches_12[i][0].distance * 1.0/pmatches_12[i][1].distance < static_cast<float>(lc_paras.ratioMax))
                    {
                        common_pt++;
                        // save data for optimization
                        Vector3d P0 = kf0->lm_3d[lr_qdx];
                        //                        double d = kf0->lm_depth[lr_qdx];
                        //                        Vector2d pl_map = kf0->lm_2d[lr_qdx];
                        //                        double x = (pl_map(0)-dc.cam0_cx)/dc.cam0_fx*d;
                        //                        double y = (pl_map(1)-dc.cam0_cy)/dc.cam0_fy*d;
                        //                        Vector3d P0(x,y,d);
                        Vector3f P = P0.cast<float>();
                        Vector2f pl_obs = kf1->lm_2d[lr_tdx].cast<float>();
                        cv::Point3f p3(P(0),P(1),P(2));
                        cv::Point2f p2(pl_obs(0),pl_obs(1));
                        p3d.push_back(p3);
                        p2d.push_back(p2);
                        selected_matches_.push_back(pmatches_12[i][0]);
                    }

                }

            }
            cv::Mat r_ = cv::Mat::zeros(3, 1, CV_64FC1);
            cv::Mat t_ = cv::Mat::zeros(3, 1, CV_64FC1);
            cv::Mat inliers;
            //SE3_to_rvec_tvec(kf0->T_c_w, r_ , t_ );
            //cout<<"start ransac"<<endl;
            if(p3d.size() < 5) return is_lc;
            cv::solvePnPRansac(p3d,p2d,dc.K0_rect,dc.D0_rect,r_,t_,false,100,2.0,0.99,inliers,cv::SOLVEPNP_P3P);

            for( int i = 0; i < inliers.rows; i++){
                int n = inliers.at<int>(i);
                select_match.push_back(selected_matches_[n]);
            }
            //    cout<<"selected points size: "<<p3d.size()<<" inliers size: "<<inliers.rows<<" unseletced size: "<<pmatches_12.size()<<endl;

            if(inliers.rows*1.0/p3d.size() < lc_paras.ratioRansac || inliers.rows < lc_paras.minPts ) //return is_lc;
            {

                return is_lc;
            }
            //SE3 se_ij
            se_ji = SE3_from_rvec_tvec(r_,t_);

            if(se_ji.translation().norm() < 3 && se_ji.so3().log().norm() < 1.5) is_lc = true;

            if(is_lc)
            {
              //publish loop closure frame.
              cv::Mat img_1 = kf_img_lc[kf_prev_idx_];
              cv::Mat img_2 = kf_img_lc[kf_curr_idx_];
              cv::cvtColor(img_1, img_1, CV_GRAY2RGB);
              cv::cvtColor(img_2, img_2, CV_GRAY2RGB);

              /* check how many rows are necessary for output matrix */
              int totalRows = img_1.rows >= img_2.rows ? img_1.rows : img_2.rows;
              cv::Mat outImg = cv::Mat::zeros( totalRows, img_1.cols + img_2.cols, CV_8UC3 );
              //cv::Mat outImg;// = cv::Mat::zeros( img_1.rows, img_1.cols + img_2.cols, img_1.type() );
              cv::Mat roi_left( outImg, cv::Rect( 0, 0, img_1.cols, img_1.rows ) );
              cv::Mat roi_right( outImg, cv::Rect( img_1.cols, 0, img_2.cols, img_2.rows ) );
              img_1.copyTo( roi_left );
              img_2.copyTo( roi_right );
              int offset = img_1.cols;


              for ( size_t counter = 0; counter < select_match.size(); counter++ )
              {
                size_t lr_qdx = static_cast<size_t>(select_match[counter].queryIdx);
                size_t lr_tdx = static_cast<size_t>(select_match[counter].trainIdx);
                cv::Scalar matchColorRGB;
                matchColorRGB = cv::Scalar( 255, 0, 0 );
                Vector2f P_left  = kf0->lm_2d[lr_qdx].cast<float>();
                Vector2f P_right = kf1->lm_2d[lr_tdx].cast<float>();
                cv::Point2f p_left(P_left(0),P_left(1));
                cv::Point2f p_right(P_right(0) + offset,P_right(1));
                cv::line( outImg, p_left, p_right, matchColorRGB, 2 );

              }
              sensor_msgs::ImagePtr lc_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outImg).toImageMsg();
              loop_closure_pub.publish(lc_msg);



            }

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
        // cout<<"first and last id in the loop: "<<kf_prev_idx<<" "<<kf_curr_idx<<endl;


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

        // cout<<"start to insert vertices "<<endl;
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
        //cout<<"start to insert adjacent edge "<<endl;


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
        //cout<<"start to insert loop edge "<<endl;

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
        for(int i = 0; i < kf_prev_idx; i++)
          path_lc_pub->pubPathT_w_c(kf_map_lc[i]->T_c_w.inverse(), kf_map_lc[i]->t);

        for (auto kf_it = kf_list.begin(); kf_it != kf_list.end(); kf_it++)
        {
            g2o::VertexSE3 *v_se3 = static_cast<g2o::VertexSE3 *>(optimizer.vertex((*kf_it)));
            g2o::SE3Quat Twc_g2o = v_se3->estimateAsSE3Quat();
            SE3 Tcw1 = kf_map_lc[static_cast<size_t>(*kf_it)]->T_c_w;

            SE3 Tw2c = SE3_from_g2o(Twc_g2o);
            SE3 Tw2_w1 =Tw2c*Tcw1;// transorm from previous to current from odom to map
            SE3 Tw1_w2 = Tw2_w1.inverse();

            kf_map_lc[static_cast<size_t>(*kf_it)]->T_c_w = Tw2c.inverse();
            path_lc_pub->pubPathT_w_c(Tw2c,kf_map_lc[static_cast<size_t>(*kf_it)]->t);
            //cout<<"after g2o pgo: ";
            //cout<<Tcw_corr<<endl;
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
        if(msg->command==KFMSG_CMD_RESET_LM)
            return;
        sim_vec.clear();
        select_match.clear();

        //STEP1: Unpack and construct KeyFrameLC tructure
        //STEP1.1 Unpack
        // [1]kf.frame_id
        // [2]kf.T_c_w_odom
        // [3]kf.T_c_w_odom
        KeyFrameLC kf;
        cv::Mat img0_unpack, img1_unpack;
        vector<int64_t> lm_id_unpack;
        vector<Vec3> lm_3d_unpack;
        vector<Vec2> lm_2d_unpack;
        vector<cv::Mat>     lm_descriptor_unpack;
        int lm_count_unpack;
        KeyFrameMsg::unpack(msg,kf.frame_id,img0_unpack,img1_unpack,lm_count_unpack,
                            lm_id_unpack,lm_2d_unpack,lm_3d_unpack,lm_descriptor_unpack,kf.T_c_w_odom,kf.t);
        SE3 T_map_odom;
        Quaterniond q_tf;
        Vec3        t_tf;
        if(1)//Visualization and publish transformation between map and odom
        {

            T_map_odom = T_odom_map.inverse();
            q_tf = T_map_odom.so3().unit_quaternion();
            t_tf = T_map_odom.translation();
            transform.setOrigin(tf::Vector3(t_tf[0],t_tf[1],t_tf[2]));
            transform.setRotation(tf::Quaternion(q_tf.x(),q_tf.y(),q_tf.z(),q_tf.w()));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
        }
        //STEP1.2 Construct KeyFrameLC
        // [1]kf.T_c_w
        // [2]kf.keyframe_id
        kf_img_lc.push_back(img0_unpack);
        BowVector kf_bv;
        SE3 loop_pose;
        kf.T_c_w = kf.T_c_w_odom*T_odom_map;
        kf.keyframe_id = kf_id++;
        path_lc_pub->pubPathT_c_w(kf.T_c_w,kf.t);

        //STEP1.3 Extract-Compute_descriptors
        vector<cv::KeyPoint> ORBFeatures;
        cv::Mat ORBDescriptorsL;
        vector<cv::Mat> ORBDescriptors;
        ORBFeatures.clear();
        ORBDescriptors.clear();
        cv::Ptr<cv::ORB> orb = cv::ORB::create(500,1.2f,8,31,0,2, cv::ORB::HARRIS_SCORE,31,20);
        orb->detectAndCompute(img0_unpack,cv::Mat(),ORBFeatures,ORBDescriptorsL);
        descriptors_to_vecDesciptor(ORBDescriptorsL,ORBDescriptors);
        kf.lm_descriptor = ORBDescriptors;

        //STEP1.4 Construct KeyFrameLC
        //Compute bow vector
        // [1]kf.kf_bv
        voc.transform(kf.lm_descriptor,kf_bv);
        kf.kf_bv = kf_bv;

        //STEP1.5 Construct KeyFrameLC
        //Recover 3d information
        // [1]kf.kf_bv
        vector<Vec3> lm_3d;
        vector<Vec2> lm_2d;
        vector<double> lm_d;
        vector<bool> lm_3d_mask;
        switch(dc.cam_type)
        {
        case STEREO_RECT:
        {
            //cout << "here" << endl;
            //track to another image
            std::vector<cv::Point2f> lm_img0, lm_img1;
            vector<float>   err;
            vector<unsigned char> status;
            cv::KeyPoint::convert(ORBFeatures,lm_img0);
            lm_img1 = lm_img0;
            //cout << lm_img0.size() << endl;
            cv::calcOpticalFlowPyrLK(img0_unpack, img1_unpack,
                                     lm_img0, lm_img1,
                                     status, err, cv::Size(31,31),5,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.001),
                                     cv::OPTFLOW_USE_INITIAL_FLOW);

            //triangulation
            for(size_t i=0; i<status.size(); i++)
            {
                if(status.at(i)==1)
                {
                    Vec3 pt3d_c;
                    if(Triangulation::trignaulationPtFromStereo(Vec2(lm_img0.at(i).x,lm_img0.at(i).y),
                                                                Vec2(lm_img1.at(i).x,lm_img1.at(i).y),
                                                                dc.P0_,
                                                                dc.P1_,
                                                                pt3d_c))
                    {
                        lm_2d.push_back(Vec2(lm_img0.at(i).x,lm_img0.at(i).y));
                        lm_d.push_back(0.0);
                        lm_3d.push_back(pt3d_c);
                        lm_3d_mask.push_back(true);
                        continue;
                    }
                    else
                    {

                        lm_2d.push_back(Vec2(0,0));
                        lm_d.push_back(0.0);
                        lm_3d.push_back(Vec3(0,0,0));
                        lm_3d_mask.push_back(false);
                        continue;
                    }
                }else
                {
                    lm_2d.push_back(Vec2(0,0));
                    lm_d.push_back(0.0);
                    lm_3d.push_back(Vec3(0,0,0));
                    lm_3d_mask.push_back(false);
                    continue;
                }
            }
            break;
        }
        case STEREO_UNRECT:
        {
            //track to another image
            //go to undistor plane
            //triangulation
            break;
        }
        case DEPTH_D435:
        {
            for(size_t i = 0; i<ORBFeatures.size();i++)
            {
                cv::Point2f cvtmp = ORBFeatures[i].pt;
                Vec2 tmp(cvtmp.x,cvtmp.y);
                double d = (img1_unpack.at<ushort>(cvtmp))/1000;
                if(d>=0.3&&d<=10)
                {
                    Vec3 p3d((tmp.x()-dc.cam0_cx)/dc.cam0_fx*d,
                             (tmp.y()-dc.cam0_cy)/dc.cam0_fy*d,
                             d);
                    lm_2d.push_back(tmp);
                    lm_d.push_back(d);
                    lm_3d.push_back(p3d);
                    lm_3d_mask.push_back(true);
                }else
                {
                    lm_2d.push_back(Vec2(0,0));
                    lm_d.push_back(0.0);
                    lm_3d.push_back(Vec3(0,0,0));
                    lm_3d_mask.push_back(false);
                }
            }
            break;
        }
        }

        //STEP1.6 Construct KeyFrameLC
        // [1]kf.lm_2d
        // [2]kf.lm_d
        // [3]kf.lm_descriptor
        // [4]kf.lm_count
        //pass feature and descriptor
        kf.lm_2d = lm_2d;
        kf.lm_3d = lm_3d;
        kf.lm_depth = lm_d;
        for(int i=lm_3d_mask.size()-1; i>=0; i--)
        {
            if(lm_3d_mask.at(i)==false)
            {
                kf.lm_descriptor.erase(kf.lm_descriptor.begin()+i);
                kf.lm_2d.erase(kf.lm_2d.begin()+i);
                kf.lm_3d.erase(kf.lm_3d.begin()+i);
            }
        }
        kf.lm_count = static_cast<int>(kf.lm_2d.size());
        lm_2d.clear();
        lm_3d.clear();
        lm_d.clear();


        //STEP2 add kf to list
        shared_ptr<KeyFrameLC> kf_lc_ptr =std::make_shared<KeyFrameLC>(kf);
        kf_map_lc.push_back(kf_lc_ptr);
        kfbv_map.push_back(kf_bv);
        expandGraph();

        //STEP3 search similar image in the list
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
        if(kf_map_lc.size()%10 == 0)
        {
            ofstream sim_txt("/home/lsgi/out/sim_mat.txt");
            sim_txt.precision(5);
            for(uint64_t i = 0; i < kf_map_lc.size(); i++)
            {
                for(uint64_t j = 0; j < kf_map_lc.size(); j++)
                {
                    sim_txt<<sim_matrix[i][j]<<" ";
                }
                sim_txt<<endl;
            }
            sim_txt.close();
        }
        if(kf_id < 50)
        {
            //cout<<"KF number is less than 50. Return."<<endl;
            return;
        }
        uint64_t kf_prev_idx;
        bool is_lc_candidate = isLoopCandidate(kf_prev_idx);
        if(!is_lc_candidate)
        {
            //cout<<"no loop candidate."<<endl;
            return;
        }
        else
        {
            //cout<<"has loop candidate."<<endl;
        }
        bool is_lc = false;
        uint64_t kf_curr_idx = kf_map_lc.size()-1;
        kf_prev_idx_ = kf_prev_idx;
        kf_curr_idx_ = kf_curr_idx;
        is_lc = isLoopClosureKF(kf_map_lc[kf_prev_idx], kf_map_lc[kf_curr_idx], loop_pose);
        if(!is_lc)
        {
            //cout<<"Geometry test fails."<<endl;
            return;
        }
        else {
            //cout<<"Pass geometry test."<<endl;
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
                printf("\033[1;31m optimize time: %lf \033[0m ", pgo.dT_ms());
                last_pgo_id = static_cast<int>(kf_curr_idx);
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

        lc_paras.lcKFStart    = getIntVariableFromYaml(configFilePath,"lcKFStart");
        lc_paras.lcKFDist     = getIntVariableFromYaml(configFilePath,"lcKFDist");
        lc_paras.lcKFMaxDist  = getIntVariableFromYaml(configFilePath,"lcKFMaxDist");
        lc_paras.lcKFLast     = getIntVariableFromYaml(configFilePath,"lcKFLast");
        lc_paras.lcNKFClosest = getIntVariableFromYaml(configFilePath,"lcNKFClosest");
        lc_paras.minPts       = getIntVariableFromYaml(configFilePath,"minPts");
        lc_paras.ratioMax     = getDoubleVariableFromYaml(configFilePath,"ratioMax");
        lc_paras.ratioRansac  = getDoubleVariableFromYaml(configFilePath,"ratioRansac");
        lc_paras.minScore     = getDoubleVariableFromYaml(configFilePath,"minScore");

        //#define VI_TYPE_D435I_DEPTH        (0)
        //#define VI_TYPE_EUROC_MAV          (1)
        //#define VI_TYPE_D435_DEPTH_PIXHAWK (2)
        //#define VI_TYPE_D435I_STEREO       (3)
        //#define VI_TYPE_KITTI_STEREO       (4)
        int vi_type_from_yaml = getIntVariableFromYaml(configFilePath,"type_of_vi");

        if(vi_type_from_yaml == VI_TYPE_D435I_DEPTH || vi_type_from_yaml == VI_TYPE_D435_DEPTH_PIXHAWK)
        {
            int w = getIntVariableFromYaml(configFilePath,                    "image_width");
            int h = getIntVariableFromYaml(configFilePath,                    "image_height");
            cv::Mat K0_rect = cameraMatrixFromYamlIntrinsics(configFilePath,  "cam0_intrinsics");
            double depth_factor = getDoubleVariableFromYaml(configFilePath,   "depth_factor");
            Mat4x4 mat_imu_cam  = Mat44FromYaml(configFilePath,               "T_imu_cam0");
            dc.setDepthCamInfo(w,
                               h,
                               K0_rect.at<double>(0,0),//fx
                               K0_rect.at<double>(1,1),//fy
                               K0_rect.at<double>(0,2),//cx
                               K0_rect.at<double>(1,2),
                               depth_factor,
                               DEPTH_D435);
        }
        if(vi_type_from_yaml == VI_TYPE_D435I_STEREO)
        {
            int w = getIntVariableFromYaml(configFilePath,             "image_width");
            int h = getIntVariableFromYaml(configFilePath,             "image_height");
            cv::Mat K0 = cameraMatrixFromYamlIntrinsics(configFilePath,"cam0_intrinsics");
            cv::Mat D0 = distCoeffsFromYaml(configFilePath,            "cam0_distortion_coeffs");
            cv::Mat K1 = cameraMatrixFromYamlIntrinsics(configFilePath,"cam1_intrinsics");
            cv::Mat D1 = distCoeffsFromYaml(configFilePath,            "cam1_distortion_coeffs");
            Mat4x4  mat_imu_cam  = Mat44FromYaml(configFilePath,       "T_imu_cam0");
            Mat4x4  mat_cam0_cam1  = Mat44FromYaml(configFilePath,     "T_cam0_cam1");
            SE3 T_i_c0 = SE3(mat_imu_cam.topLeftCorner(3,3),
                             mat_imu_cam.topRightCorner(3,1));
            SE3 T_c0_c1 = SE3(mat_cam0_cam1.topLeftCorner(3,3),
                              mat_cam0_cam1.topRightCorner(3,1));
            SE3 T_c1_c0 = T_c0_c1.inverse();
            Mat3x3 R_ = T_c1_c0.rotation_matrix();
            Vec3   T_ = T_c1_c0.translation();
            cv::Mat R__ = (cv::Mat1d(3, 3) <<
                           R_(0,0), R_(0,1), R_(0,2),
                           R_(1,0), R_(1,1), R_(1,2),
                           R_(2,0), R_(2,1), R_(2,2));
            cv::Mat T__ = (cv::Mat1d(3, 1) << T_(0), T_(1), T_(2));
            cv::Mat R0,R1,P0,P1,Q;
            cv::stereoRectify(K0,D0,K1,D1,cv::Size(w,h),R__,T__,
                              R0,R1,P0,P1,Q,
                              cv::CALIB_ZERO_DISPARITY,0,cv::Size(w,h));
            cv::Mat K0_rect = P0.rowRange(0,3).colRange(0,3);
            cv::Mat K1_rect = P1.rowRange(0,3).colRange(0,3);
            cv::Mat D0_rect,D1_rect;
            D1_rect = D0_rect = (cv::Mat1d(4, 1) << 0,0,0,0);
            dc.setSteroCamInfo(w,h,
                               K0, D0, K0_rect, D0_rect, R0, P0,
                               K1, D1, K1_rect, D1_rect, R1, P1,
                               T_c0_c1,STEREO_RECT);
        }
        if(vi_type_from_yaml == VI_TYPE_EUROC_MAV)
        {
            int w = getIntVariableFromYaml(configFilePath,             "image_width");
            int h = getIntVariableFromYaml(configFilePath,             "image_height");
            cv::Mat K0 = cameraMatrixFromYamlIntrinsics(configFilePath,"cam0_intrinsics");
            cv::Mat D0 = distCoeffsFromYaml(configFilePath,            "cam0_distortion_coeffs");
            cv::Mat K1 = cameraMatrixFromYamlIntrinsics(configFilePath,"cam1_intrinsics");
            cv::Mat D1 = distCoeffsFromYaml(configFilePath,            "cam1_distortion_coeffs");
            Mat4x4  mat_mavimu_cam0  = Mat44FromYaml(configFilePath,   "T_mavimu_cam0");
            Mat4x4  mat_mavimu_cam1  = Mat44FromYaml(configFilePath,   "T_mavimu_cam1");
            Mat4x4  mat_i_mavimu  = Mat44FromYaml(configFilePath,      "T_imu_mavimu");
            SE3 T_mavi_c0 = SE3(mat_mavimu_cam0.topLeftCorner(3,3),
                                mat_mavimu_cam0.topRightCorner(3,1));
            SE3 T_mavi_c1 = SE3(mat_mavimu_cam1.topLeftCorner(3,3),
                                mat_mavimu_cam1.topRightCorner(3,1));
            SE3 T_c0_c1 = T_mavi_c0.inverse()*T_mavi_c1;
            SE3 T_c1_c0 = T_c0_c1.inverse();
            SE3 T_i_mavi = SE3(mat_i_mavimu.topLeftCorner(3,3),mat_i_mavimu.topRightCorner(3,1));
            SE3 T_i_c0 = T_i_mavi*T_mavi_c0;
            Mat3x3 R_ = T_c1_c0.rotation_matrix();
            Vec3   T_ = T_c1_c0.translation();
            cv::Mat R__ = (cv::Mat1d(3, 3) <<
                           R_(0,0), R_(0,1), R_(0,2),
                           R_(1,0), R_(1,1), R_(1,2),
                           R_(2,0), R_(2,1), R_(2,2));
            cv::Mat T__ = (cv::Mat1d(3, 1) << T_(0), T_(1), T_(2));
            cv::Mat R0,R1,P0,P1,Q;
            cv::stereoRectify(K0,D0,K1,D1,cv::Size(w,h),R__,T__,
                              R0,R1,P0,P1,Q,
                              cv::CALIB_ZERO_DISPARITY,0,cv::Size(w,h));
            cv::Mat K0_rect = P0.rowRange(0,3).colRange(0,3);
            cv::Mat K1_rect = P1.rowRange(0,3).colRange(0,3);
            cv::Mat D0_rect,D1_rect;
            D1_rect = D0_rect = (cv::Mat1d(4, 1) << 0,0,0,0);
            dc.setSteroCamInfo(w,h,
                               K0, D0, K0_rect, D0_rect, R0, P0,
                               K1, D1, K1_rect, D1_rect, R1, P1,
                               T_c0_c1,STEREO_UNRECT);
        }
        if(vi_type_from_yaml == VI_TYPE_KITTI_STEREO)
        {
            int w = getIntVariableFromYaml(configFilePath,             "image_width");
            int h = getIntVariableFromYaml(configFilePath,             "image_height");
            Mat4x4  P0_ = Mat44FromYaml(configFilePath,"cam0_projection_matrix");
            Mat4x4  P1_ = Mat44FromYaml(configFilePath,"cam1_projection_matrix");
            Mat4x4  K_inverse;
            K_inverse.fill(0);
            Mat3x3 K = P0_.topLeftCorner(3,3);
            K_inverse.topLeftCorner(3,3) = K.inverse();
            cout << "K_inverse" << endl << K_inverse << endl;
            Mat4x4 mat_T_c0_c1 = K_inverse*P1_;
            mat_T_c0_c1.topLeftCorner(3,3).setIdentity();
            SE3 T_c0_c1(mat_T_c0_c1.topLeftCorner(3,3),mat_T_c0_c1.topRightCorner(3,1));
            cv::Mat P0 = (cv::Mat1d(3, 4) <<
                          P0_(0,0), P0_(0,1), P0_(0,2), P0_(0,3),
                          P0_(1,0), P0_(1,1), P0_(1,2), P0_(1,3),
                          P0_(2,0), P0_(2,1), P0_(2,2), P0_(2,3));
            cv::Mat P1 = (cv::Mat1d(3, 4) <<
                          P1_(0,0), P1_(0,1), P1_(0,2), P1_(0,3),
                          P1_(1,0), P1_(1,1), P1_(1,2), P1_(1,3),
                          P1_(2,0), P1_(2,1), P1_(2,2), P1_(2,3));
            cv::Mat K0,K1,K0_rect,K1_rect;
            cv::Mat D0,D1,D0_rect,D1_rect;
            D1_rect = D0_rect = D1 = D0 =(cv::Mat1d(4, 1) << 0,0,0,0);
            K0 = K1 = K0_rect = P0.rowRange(0,3).colRange(0,3);
            K1_rect = P1.rowRange(0,3).colRange(0,3);
            dc.setSteroCamInfo(w,h,
                               K0, D0, K0_rect, D0_rect, (cv::Mat1d(3, 3) << 1,0,0,0,1,0,0,0,1), P0,
                               K1, D1, K1_rect, D1_rect, (cv::Mat1d(3, 3) << 1,0,0,0,1,0,0,0,1), P1,
                               T_c0_c1,STEREO_RECT);
        }

        string vocFile;
        nh.getParam("/voc", vocFile);
        Vocabulary vocTmp(vocFile);
        cout<<"voc begin: "<<endl;
        voc = vocTmp;
        kf_id = 0;
        Database dbTmp(voc, true, 0);
        db = dbTmp;
        path_lc_pub  = new RVIZPath(nh,"/vision_path_lc_all","map");
        image_transport::ImageTransport it(nh);
        loop_closure_pub = it.advertise("/loop_closure_img",1);
        sub_kf = nh.subscribe<flvis::KeyFrame>(
                    "/vo_kf",
                    2,
                    boost::bind(&LoopClosingNodeletClass::frame_callback, this, _1));

    }




};//class LoopClosingNodeletClass
}//namespace flvis_ns



PLUGINLIB_EXPORT_CLASS(flvis_ns::LoopClosingNodeletClass, nodelet::Nodelet)

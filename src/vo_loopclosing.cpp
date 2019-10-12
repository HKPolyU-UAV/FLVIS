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

// DBoW3
#include "../3rdPartLib/DBow3/src/DBoW3.h"
#include "../3rdPartLib/DBow3/src/DescManip.h"


using namespace DBoW3;
using namespace cv;
using namespace std;

namespace vo_nodelet_ns
{

struct KeyFrameLC
{
  KeyFrameLC(){}
  int64_t kf_id;
  vector<int64_t> lm_id;
  vector<Vec2> lm_2d;
  vector<Vec3> lm_3d;
  vector<Mat> lm_descriptors;
  SE3 T_c_w;
  BowVector kf_bv;

};
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
#define lcNKFClosest (4)
#define ratioMax (0.7)
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

    static uint64_t kf_id;

    //DBow related para
    Vocabulary voc;
    Database db;// faster search
    vector<vector<double>> sim_matrix;//for sim visualization
    //KF database
    vector<shared_ptr<KeyFrameLC>> kf_map;



    bool isLoopCandidate(uint64_t kf_curr_idx, uint64_t &kf_prev_idx)
    {
      bool is_lc_candidate = false;
      vector<Vector2d> max_sim_mat;
      for (uint64_t i = 0; i < kf_map.size() - lcKFDist; i++)
      {
          if (kf_map[i] != nullptr)
          {
              Vector2d aux;
              aux(0) = i;
              aux(1) = sim_matrix[i][kf_curr_idx];
              max_sim_mat.push_back(aux);
          }
      }

      sort(max_sim_mat.begin(), max_sim_mat.end(), sort_simmat_by_score());

      // find the minimum score in the covisibility graph (and/or 3 previous keyframes)
      double lc_min_score = 1.0;
      for (uint64_t i = 0; i < kf_map.size(); i++)
      {
        if (i >= kf_map.size() -lcKFDist)
        {
          double score_i = sim_matrix[i][kf_curr_idx];
          if (score_i < lc_min_score && score_i > 0.001)
          lc_min_score = score_i;
        }
      }

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
       return is_lc_candidate;
    }

    bool isLoopClosure(shared_ptr<KeyFrameLC> kf0, shared_ptr<KeyFrameLC> kf1,SE3 &kf01)
    {
      int common_pt = 0;

      if (!(kf1->lm_descriptors.size() == 0) && !(kf0->lm_descriptors.size() == 0))
      {

          BFMatcher *bfm = new BFMatcher(NORM_HAMMING, false); // cross-check
          Mat pdesc_l1, pdesc_l2;
          vector<vector<DMatch>> pmatches_12, pmatches_21;
          // 12 and 21 matches
          vMat_to_descriptors(pdesc_l1,kf0->lm_descriptors);
          vMat_to_descriptors(pdesc_l2,kf1->lm_descriptors);
          bfm->knnMatch(pdesc_l1, pdesc_l2, pmatches_12, 2);
          bfm->knnMatch(pdesc_l2, pdesc_l1, pmatches_21, 2);

          // resort according to the queryIdx
          sort(pmatches_12.begin(), pmatches_12.end(), sort_descriptor_by_queryIdx());
          sort(pmatches_21.begin(), pmatches_21.end(), sort_descriptor_by_queryIdx());
          // bucle around pmatches
          for (int i = 0; i < pmatches_12.size(); i++)
          {
              // check if they are mutual best matches
              int lr_qdx = pmatches_12[i][0].queryIdx;
              int lr_tdx = pmatches_12[i][0].trainIdx;
              int rl_tdx = pmatches_21[lr_tdx][0].trainIdx;

              // check if they are mutual best matches and the minimum distance
              //            double dist_nn = pmatches_12[i][0].distance;
              //            double dist_12 = pmatches_12[i][0].distance / pmatches_12[i][1].distance;
              // check the f2f max disparity condition
              vector<Point3f> p3d;
              vector<Point2f> p2d;
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

              cv::Mat r_ = cv::Mat::zeros(3, 1, CV_64FC1);
              cv::Mat t_ = cv::Mat::zeros(3, 1, CV_64FC1);
              SE3_to_rvec_tvec(kf0->T_c_w, r_ , t_ );
              solvePnPRansac(p3d,p2d,cameraMatrix,distCoeffs,r_,t_,false,100,8.0,0.99,cv::noArray(),SOLVEPNP_P3P);
          }
      }
    }









    void frame_callback(const vo_nodelet::KeyFrameConstPtr& msg)
    {

        //KeyFrameLC* kf = new KeyFrameLC();
        shared_ptr<KeyFrameLC> kf;
        Mat img;
        vector<int64_t> lm_id;
        vector<Vec2> lm_2d;
        vector<Vec3> lm_3d;
        vector<Mat> lm_descriptors;
        SE3 T_c_w;
        BowVector kf_bv;
        int64_t kf_id2;
        KeyFrameMsg::unpack(msg,kf_id2,img,lm_id,lm_2d,lm_3d,lm_descriptors,T_c_w);
        voc.transform(lm_descriptors,kf_bv);
        kf->kf_id = kf_id++;
        kf->lm_id = lm_id;
        kf->lm_2d = lm_2d;
        kf->lm_3d = lm_3d;
        kf->lm_descriptors = lm_descriptors;
        kf->T_c_w = T_c_w;
        kf->kf_bv = kf_bv;
        kf_map.push_back(kf);
        //int idx = kf->kf_id;
        for (uint64_t i = 0; i < kf_map.size(); i++)
        {
          if(kf_map[i] != nullptr)
          {
            double score = voc.score(kf_bv,kf_map[i]->kf_bv);
            sim_matrix[kf_id-1][i] = score;
            sim_matrix[i][kf_id-1] = score;
          }
        }










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


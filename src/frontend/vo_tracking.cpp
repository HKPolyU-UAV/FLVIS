#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <include/tic_toc_ros.h>
#include <include/common.h>
#include <include/f2f_tracking.h>
#include <include/vi_type.h>
#include <include/rviz_frame.h>
#include <include/rviz_path.h>
#include <include/rviz_pose.h>
#include <include/rviz_odom.h>

#include <include/yamlRead.h>
#include <include/cv_draw.h>
#include <flvis/KeyFrame.h>
#include <flvis/CorrectionInf.h>
#include <include/keyframe_msg.h>
#include <include/correction_inf_msg.h>
#include <tf/transform_listener.h>


namespace flvis_ns
{


enum TYPEOFIMU{D435I,
               EuRoC_MAV,
               PIXHAWK,
               NONE};

class TrackingNodeletClass : public nodelet::Nodelet
{
public:
  TrackingNodeletClass()  {;}
  ~TrackingNodeletClass() {;}
private:
  bool is_lite_version;
  enum TYPEOFCAMERA cam_type;
  enum TYPEOFIMU imu_type;
  F2FTracking   *cam_tracker;
  //Subscribers
  message_filters::Subscriber<sensor_msgs::Image> img0_sub;
  message_filters::Subscriber<sensor_msgs::Image> img1_sub;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MyExactSyncPolicy;
  message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
  ros::Subscriber imu_sub;
  ros::Subscriber correction_inf_sub;

  //Octomap
  //OctomapFeeder* octomap_pub;
  //Visualization
  cv::Mat img0_vis;
  cv::Mat img1_vis;
  image_transport::Publisher img0_pub;
  image_transport::Publisher img1_pub;
  ros::Publisher imu_pose_pub;
  ros::Publisher vision_pose_pub;
  RVIZFrame* frame_pub;
  RVIZPath*  vision_path_pub;
  RVIZPath*  imu_path_pub;
  RVIZPath*  path_lc_pub;
  RVIZOdom*  odom_imu_pub;
  RVIZPose*  pose_imu_pub;
  KeyFrameMsg* kf_pub;
  tf::StampedTransform tranOdomMap;
  tf::TransformListener listenerOdomMap;

  virtual void onInit()
  {
    ros::NodeHandle& nh = getMTPrivateNodeHandle();
    //cv::startWindowThread(); //Bug report https://github.com/ros-perception/image_pipeline/issues/201

    //Publisher
    vision_path_pub = new RVIZPath(nh,"/vision_path","map",1,10000);
    path_lc_pub     = new RVIZPath(nh,"/vision_path_lc","map",1,10000);
    imu_path_pub    = new RVIZPath(nh,"/imu_path","map",1,400);
    frame_pub       = new RVIZFrame(nh,"/vo_camera_pose","map","/vo_curr_frame","map");
    pose_imu_pub    = new RVIZPose(nh,"/imu_pose","map");
    odom_imu_pub    = new RVIZOdom(nh,"/imu_odom","map");
    kf_pub          = new KeyFrameMsg(nh,"/vo_kf");
    //        octomap_pub  = new OctomapFeeder(nh,"/vo_octo_tracking","vo_local",1);
    //        octomap_pub->d_camera=curr_frame->d_camera;
    image_transport::ImageTransport it(nh);
    img0_pub = it.advertise("/vo_img0", 1);
    img1_pub = it.advertise("/vo_img1", 1);

    cam_tracker = new F2FTracking();
    //Load Parameter
    string configFilePath;
    nh.getParam("/yamlconfigfile",   configFilePath);
    is_lite_version = false;
    is_lite_version = getBoolVariableFromYaml(configFilePath,"is_lite_version");
    if(is_lite_version)
      cout << "flvis run in lite version" << endl;

    cout << configFilePath << endl;

    int vi_type_from_yaml = getIntVariableFromYaml(configFilePath,"type_of_vi");

    Vec3 vi_para1 = Vec3(getDoubleVariableFromYaml(configFilePath,"vifusion_para1"),
                         getDoubleVariableFromYaml(configFilePath,"vifusion_para2"),
                         getDoubleVariableFromYaml(configFilePath,"vifusion_para3"));
    Vec3 vi_para2 = Vec3(getDoubleVariableFromYaml(configFilePath,"vifusion_para4"),
                         getDoubleVariableFromYaml(configFilePath,"vifusion_para5"),
                         getDoubleVariableFromYaml(configFilePath,"vifusion_para6"));
    Vec6 vi_para;
    vi_para.head(3)=vi_para1;
    vi_para.tail(3)=vi_para2;

    Vec3 f_para1 = Vec3(getDoubleVariableFromYaml(configFilePath,"feature_para1"),//max features in a grid
                        getDoubleVariableFromYaml(configFilePath,"feature_para2"),//min features in a grid
                        getDoubleVariableFromYaml(configFilePath,"feature_para3"));//distance of features
    Vec3 f_para2 = Vec3(getDoubleVariableFromYaml(configFilePath,"feature_para4"),//goodFeaturesToTrack detector maxCorners
                        getDoubleVariableFromYaml(configFilePath,"feature_para5"),//goodFeaturesToTrack detector qualityLevel
                        getDoubleVariableFromYaml(configFilePath,"feature_para6"));//goodFeaturesToTrack detector minDistance
    Vec6 f_para;//feature related parameters
    f_para.head(3)=f_para1;
    f_para.tail(3)=f_para2;

    if(vi_type_from_yaml==VI_TYPE_D435I_DEPTH)        {cam_type=DEPTH_D435;    imu_type=D435I;}
    if(vi_type_from_yaml==VI_TYPE_EUROC_MAV)          {cam_type=STEREO_UNRECT; imu_type=EuRoC_MAV;}
    if(vi_type_from_yaml==VI_TYPE_D435_DEPTH_PIXHAWK) {cam_type=DEPTH_D435;    imu_type=PIXHAWK;}
    if(vi_type_from_yaml==VI_TYPE_D435I_STEREO)       {cam_type=STEREO_RECT;   imu_type=D435I;}
    if(vi_type_from_yaml==VI_TYPE_KITTI_STEREO)       {cam_type=STEREO_RECT;   imu_type=NONE;}


    if(vi_type_from_yaml == VI_TYPE_D435I_DEPTH || vi_type_from_yaml == VI_TYPE_D435_DEPTH_PIXHAWK)
    {
      int w = getIntVariableFromYaml(configFilePath,                    "image_width");
      int h = getIntVariableFromYaml(configFilePath,                    "image_height");
      cv::Mat K0_rect = cameraMatrixFromYamlIntrinsics(configFilePath,  "cam0_intrinsics");
      double depth_factor = getDoubleVariableFromYaml(configFilePath,   "depth_factor");
      Mat4x4 mat_imu_cam  = Mat44FromYaml(configFilePath,               "T_imu_cam0");
      DepthCamera dc;
      dc.setDepthCamInfo(w,
                         h,
                         K0_rect.at<double>(0,0),//fx
                         K0_rect.at<double>(1,1),//fy
                         K0_rect.at<double>(0,2),//cx
                         K0_rect.at<double>(1,2),
                         depth_factor,
                         DEPTH_D435);
      cam_tracker->init(dc,
                        SE3(mat_imu_cam.topLeftCorner(3,3),mat_imu_cam.topRightCorner(3,1)),
                        f_para,
                        vi_para,
                        50,
                        false);
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
                        CALIB_ZERO_DISPARITY,0,cv::Size(w,h));
      cv::Mat K0_rect = P0.rowRange(0,3).colRange(0,3);
      cv::Mat K1_rect = P1.rowRange(0,3).colRange(0,3);
      cv::Mat D0_rect,D1_rect;
      D1_rect = D0_rect = (cv::Mat1d(4, 1) << 0,0,0,0);
      DepthCamera dc;
      dc.setSteroCamInfo(w,h,
                         K0, D0, K0_rect, D0_rect, R0, P0,
                         K1, D1, K1_rect, D1_rect, R1, P1,
                         T_c0_c1,STEREO_RECT);
      cam_tracker->init(dc,
                        T_i_c0,
                        f_para,
                        vi_para,
                        50,
                        false);
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
                        CALIB_ZERO_DISPARITY,0,cv::Size(w,h));
      cv::Mat K0_rect = P0.rowRange(0,3).colRange(0,3);
      cv::Mat K1_rect = P1.rowRange(0,3).colRange(0,3);
      cv::Mat D0_rect,D1_rect;
      D1_rect = D0_rect = (cv::Mat1d(4, 1) << 0,0,0,0);
      DepthCamera dc;
      dc.setSteroCamInfo(w,h,
                         K0, D0, K0_rect, D0_rect, R0, P0,
                         K1, D1, K1_rect, D1_rect, R1, P1,
                         T_c0_c1,STEREO_UNRECT);
      cam_tracker->init(dc,
                        T_i_c0,
                        f_para,
                        vi_para,
                        0,
                        true);
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
      DepthCamera dc;
      dc.setSteroCamInfo(w,h,
                         K0, D0, K0_rect, D0_rect, (cv::Mat1d(3, 3) << 1,0,0,0,1,0,0,0,1), P0,
                         K1, D1, K1_rect, D1_rect, (cv::Mat1d(3, 3) << 1,0,0,0,1,0,0,0,1), P1,
                         T_c0_c1,STEREO_RECT);
      cam_tracker->init(dc,
                        SE3(),//dummy parameter
                        f_para,
                        vi_para,
                        0,
                        false);

    }

    img0_sub.subscribe(nh, "/vo/input_image_0", 1);
    img1_sub.subscribe(nh, "/vo/input_image_1", 1);
    correction_inf_sub = nh.subscribe<flvis::CorrectionInf>(
          "/vo_localmap_feedback",
          1,
          boost::bind(&TrackingNodeletClass::correction_feedback_callback, this, _1));
    imu_sub = nh.subscribe<sensor_msgs::Imu>(
          "/imu",
          10,
          boost::bind(&TrackingNodeletClass::imu_callback, this, _1));
    exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(2), img0_sub, img1_sub);
    exactSync_->registerCallback(boost::bind(&TrackingNodeletClass::image_input_callback, this, _1, _2));
    cout << "start tracking thread" << endl;
  }


  void imu_callback(const sensor_msgs::ImuConstPtr& msg)
  {
    //SETP1: TO ENU Frame
    Vec3 acc,gyro;
    ros::Time tstamp = msg->header.stamp;
    if(imu_type==D435I)
    {
      acc = Vec3(-msg->linear_acceleration.z,
                 msg->linear_acceleration.x,
                 msg->linear_acceleration.y);
      gyro = Vec3(msg->angular_velocity.z,
                  -msg->angular_velocity.x,
                  -msg->angular_velocity.y);
    }
    if(imu_type==EuRoC_MAV)
    {
      gyro = Vec3(msg->angular_velocity.z,
                  -msg->angular_velocity.y,
                  msg->angular_velocity.x);
      acc = Vec3(-msg->linear_acceleration.z,
                 msg->linear_acceleration.y,
                 -msg->linear_acceleration.x);
    }
    if(imu_type==PIXHAWK)
    {
      acc = Vec3(-msg->linear_acceleration.x,
                 -msg->linear_acceleration.y,
                 -msg->linear_acceleration.z);
      gyro = Vec3(msg->angular_velocity.x,
                  msg->angular_velocity.y,
                  msg->angular_velocity.z);
    }



    //nur for test
    //acc = Vec3(4.55,0.3,-7.88);
    //gyro =  Vec3(0.001,0.002,0.003);
    Quaterniond q_w_i;
    Vec3        pos_w_i, vel_w_i;
    cam_tracker->imu_feed(tstamp.toSec(),acc,gyro,
                          q_w_i,pos_w_i,vel_w_i);
    pose_imu_pub->pubPose(q_w_i,pos_w_i,tstamp);
    odom_imu_pub->pubOdom(q_w_i,pos_w_i,vel_w_i,tstamp);
    imu_path_pub->pubPathT_w_c(SE3(q_w_i,pos_w_i),tstamp);
  }

  void correction_feedback_callback(const flvis::CorrectionInf::ConstPtr& msg)
  {
    //unpacking and update the structure
    CorrectionInfStruct correction_inf;
    CorrectionInfMsg::unpack(msg,
                             correction_inf.frame_id,
                             correction_inf.T_c_w,
                             correction_inf.lm_count,
                             correction_inf.lm_id,
                             correction_inf.lm_3d,
                             correction_inf.lm_outlier_count,
                             correction_inf.lm_outlier_id);
  }

  void image_input_callback(const sensor_msgs::ImageConstPtr & img0_Ptr,
                            const sensor_msgs::ImageConstPtr & img1_Ptr)
  {
    //tic_toc_ros tt_cb;
    ros::Time tstamp = img0_Ptr->header.stamp;

    cv_bridge::CvImagePtr cvbridge_img0  = cv_bridge::toCvCopy(img0_Ptr, img0_Ptr->encoding);
    cv_bridge::CvImagePtr cvbridge_img1  = cv_bridge::toCvCopy(img1_Ptr, img1_Ptr->encoding);
    bool newkf;//new key frame
    bool reset_cmd;//reset command to localmap node
    this->cam_tracker->image_feed(tstamp.toSec(),
                                  cvbridge_img0->image,
                                  cvbridge_img1->image,
                                  newkf,
                                  reset_cmd);
    if(newkf) kf_pub->pub(*cam_tracker->curr_frame,tstamp);
    if(reset_cmd) kf_pub->cmdLMResetPub(ros::Time(tstamp));
    frame_pub->pubFramePtsPoseT_c_w(this->cam_tracker->curr_frame->getValid3dPts(),
                                    this->cam_tracker->curr_frame->T_c_w,
                                    tstamp);
    vision_path_pub->pubPathT_c_w(this->cam_tracker->curr_frame->T_c_w,tstamp);
    SE3 T_map_c =SE3();
    try{
      listenerOdomMap.lookupTransform("map","odom",ros::Time(0), tranOdomMap);
      tf::Vector3 tf_t= tranOdomMap.getOrigin();
      tf::Quaternion tf_q = tranOdomMap.getRotation();
      SE3 T_map_odom(Quaterniond(tf_q.w(),tf_q.x(),tf_q.y(),tf_q.z()),
                     Vec3(tf_t.x(),tf_t.y(),tf_t.z()));
      T_map_c = T_map_odom*this->cam_tracker->curr_frame->T_c_w.inverse();
      path_lc_pub->pubPathT_w_c(T_map_c,tstamp);
    }
    catch (tf::TransformException ex)
    {
      //cout<<"no transform between map and odom yet."<<endl;
    }
    if(!is_lite_version)
    {
      cvtColor(cam_tracker->curr_frame->img0,img0_vis,CV_GRAY2BGR);
      if(cam_type==DEPTH_D435)
      {
        drawFrame(img0_vis,*this->cam_tracker->curr_frame,1,6);
        visualizeDepthImg(img1_vis,*this->cam_tracker->curr_frame);
      }
      if(cam_type==STEREO_UNRECT || cam_type==STEREO_RECT)
      {
        drawFrame(img0_vis,*this->cam_tracker->curr_frame,1,11);
        cvtColor(cam_tracker->curr_frame->img1,img1_vis,CV_GRAY2BGR);
        drawFlow(img0_vis,
                 this->cam_tracker->curr_frame->flow_last,
                 this->cam_tracker->curr_frame->flow_curr);
        drawFlow(img1_vis,
                 this->cam_tracker->curr_frame->flow_0,
                 this->cam_tracker->curr_frame->flow_1);
      }
      sensor_msgs::ImagePtr img0_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img0_vis).toImageMsg();
      sensor_msgs::ImagePtr img1_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img1_vis).toImageMsg();
      img0_pub.publish(img0_msg);
      img1_pub.publish(img1_msg);
    }
  }//image_input_callback(const sensor_msgs::ImageConstPtr & imgPtr, const sensor_msgs::ImageConstPtr & depthImgPtr)

};//class TrackingNodeletClass
}//namespace flvis_ns

PLUGINLIB_EXPORT_CLASS(flvis_ns::TrackingNodeletClass, nodelet::Nodelet)



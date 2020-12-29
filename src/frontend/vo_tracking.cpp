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
//#include <include/octomap_feeder.h>

namespace flvis_ns
{

enum TYPEOFIMU{D435I,
               EuRoC_MAV,
               PIXHAWK};

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
        vision_path_pub = new RVIZPath(nh,"/vision_path","map",1,3000);
        path_lc_pub     = new RVIZPath(nh,"/vision_path_lc","map",1,3000);
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
        int image_width  = getIntVariableFromYaml(configFilePath,"image_width");
        int image_height = getIntVariableFromYaml(configFilePath,"image_height");
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
        cv::Mat cam0_cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath,"cam0_intrinsics");
        cv::Mat cam0_distCoeffs   = distCoeffsFromYaml(configFilePath,"cam0_distortion_coeffs");
        //        cout << "image_width :" << image_width << endl;
        //        cout << "image_height:" << image_height << endl;
        //        cout << "cam0_cameraMatrix:" << endl << cam0_cameraMatrix << endl;
        //        cout << "cam0_distCoeffs  :" << endl << cam0_distCoeffs << endl;
        if(vi_type_from_yaml==0)
        {
            cam_type=DEPTH_D435;
            imu_type=D435I;
        }
        if(vi_type_from_yaml==1)
        {
            cam_type=STEREO_EuRoC_MAV;
            imu_type=EuRoC_MAV;
        }
        if(vi_type_from_yaml==2)
        {
            cam_type=DEPTH_D435;
            imu_type=PIXHAWK;
        }
        if(vi_type_from_yaml==3)
        {
            cam_type=STEREO_D435;
            imu_type=D435I;
        }
        if(cam_type==DEPTH_D435)
        {
            Mat4x4  mat_imu_cam  = Mat44FromYaml(configFilePath,"T_imu_cam0");
            cam_tracker->init(image_width,
                              image_height,
                              cam0_cameraMatrix,
                              cam0_distCoeffs,
                              SE3(mat_imu_cam.topLeftCorner(3,3),mat_imu_cam.topRightCorner(3,1)),
                              f_para,
                              vi_para);
        }
        if(cam_type==STEREO_D435)
        {
            cv::Mat cam1_cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath,"cam1_intrinsics");
            cv::Mat cam1_distCoeffs   = distCoeffsFromYaml(configFilePath,"cam1_distortion_coeffs");
            Mat4x4  mat_imu_cam  = Mat44FromYaml(configFilePath,"T_imu_cam0");
            SE3 T_i_c0 = SE3(mat_imu_cam.topLeftCorner(3,3),
                             mat_imu_cam.topRightCorner(3,1));
            Mat4x4  mat_cam0_cam1  = Mat44FromYaml(configFilePath,"T_cam0_cam1");
            SE3 T_c0_c1 = SE3(mat_cam0_cam1.topLeftCorner(3,3),
                              mat_cam0_cam1.topRightCorner(3,1));
            cam_tracker->init(image_width,image_height,
                              cam0_cameraMatrix,cam0_distCoeffs,
                              T_i_c0,
                              f_para,
                              vi_para,
                              STEREO_D435,
                              1.0,
                              cam1_cameraMatrix,cam1_distCoeffs,
                              T_c0_c1);
        }
        if(cam_type==STEREO_EuRoC_MAV)
        {
            Mat4x4  mat_mavimu_cam0  = Mat44FromYaml(configFilePath,"T_mavimu_cam0");
            SE3 T_mavi_c0 = SE3(mat_mavimu_cam0.topLeftCorner(3,3),
                                mat_mavimu_cam0.topRightCorner(3,1));
            cv::Mat cam1_cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath,"cam1_intrinsics");
            cv::Mat cam1_distCoeffs   = distCoeffsFromYaml(configFilePath,"cam1_distortion_coeffs");
            Mat4x4  mat_mavimu_cam1  = Mat44FromYaml(configFilePath,"T_mavimu_cam1");
            SE3 T_mavi_c1 = SE3(mat_mavimu_cam1.topLeftCorner(3,3),
                                mat_mavimu_cam1.topRightCorner(3,1));
            SE3 T_c0_c1 = T_mavi_c0.inverse()*T_mavi_c1;
            Mat4x4  mat_i_mavimu  = Mat44FromYaml(configFilePath,"T_imu_mavimu");
            SE3 T_i_mavi = SE3(mat_i_mavimu.topLeftCorner(3,3),mat_i_mavimu.topRightCorner(3,1));
            SE3 T_i_c0 = T_i_mavi*T_mavi_c0;
            cam_tracker->init(image_width,image_height,
                              cam0_cameraMatrix,cam0_distCoeffs,
                              T_i_c0,
                              f_para,
                              vi_para,
                              STEREO_EuRoC_MAV,
                              1.0,
                              cam1_cameraMatrix,cam1_distCoeffs,
                              T_c0_c1);
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
            if(cam_type==STEREO_EuRoC_MAV || cam_type==STEREO_D435)
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



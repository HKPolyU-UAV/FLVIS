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
#include <include/yamlRead.h>
#include <include/cv_draw.h>
#include <flvis/KeyFrame.h>
#include <flvis/CorrectionInf.h>
#include <include/keyframe_msg.h>
#include <include/correction_inf_msg.h>
#include <include/octomap_feeder.h>
#include <tf/transform_listener.h>


namespace flvis_ns
{
class TrackingNodeletClass : public nodelet::Nodelet
{
public:
    TrackingNodeletClass()  {;}
    ~TrackingNodeletClass() {;}
private:

    enum TYPEOFCAMERA cam_type;
    F2FTracking   *cam_tracker;
    //Subscribers
    message_filters::Subscriber<sensor_msgs::Image> img0_sub;
    message_filters::Subscriber<sensor_msgs::Image> img1_sub;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MyExactSyncPolicy;
    message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
    ros::Subscriber imu_sub;
    ros::Subscriber correction_inf_sub;

    //Octomap
    OctomapFeeder* octomap_pub;
    //Visualization
    cv::Mat img_vis;
    cv::Mat dimg_vis;
    image_transport::Publisher img_pub;
    image_transport::Publisher dimg_pub;
    ros::Publisher imu_pose_pub;
    ros::Publisher vision_pose_pub;
    RVIZFrame* frame_pub;
    RVIZPath*  path_pub;
    RVIZPath*  path_lc_pub;
    RVIZPose*  pose_imu_pub;
    KeyFrameMsg* kf_pub;
    tf::StampedTransform tranOdomMap;
    tf::TransformListener listenerOdomMap;

    virtual void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        //cv::startWindowThread(); //Bug report https://github.com/ros-perception/image_pipeline/issues/201

        //Publisher
        path_pub     = new RVIZPath(nh,"/vo_path","map",1,3000);
        path_lc_pub  = new RVIZPath(nh,"/vo_path_lc","map",1,3000);
        frame_pub    = new RVIZFrame(nh,"/vo_camera_pose","map","/vo_curr_frame","map");
        pose_imu_pub = new RVIZPose(nh,"/imu_pose","map");
        kf_pub       = new KeyFrameMsg(nh,"/vo_kf");
        //        octomap_pub  = new OctomapFeeder(nh,"/vo_octo_tracking","vo_local",1);
        //        octomap_pub->d_camera=curr_frame->d_camera;
        image_transport::ImageTransport it(nh);
        img_pub = it.advertise("/vo_img", 1);
        dimg_pub = it.advertise("/vo_dimg", 1);

        cam_tracker = new F2FTracking();
        //Load Parameter
        string configFilePath;
        nh.getParam("/yamlconfigfile",   configFilePath);
        cout << configFilePath << endl;
        int cam_type_from_yaml = getIntVariableFromYaml(configFilePath,"type_of_cam");
        if(cam_type_from_yaml==0) cam_type=DEPTH_D435I;
        if(cam_type_from_yaml==1) cam_type=STEREO_EuRoC_MAV;
        if(cam_type==DEPTH_D435I)
        {
            int image_width  = getIntVariableFromYaml(configFilePath,"image_width");
            int image_height = getIntVariableFromYaml(configFilePath,"image_height");
            cv::Mat cam0_cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath,"cam0_intrinsics");
            cv::Mat cam0_distCoeffs   = distCoeffsFromYaml(configFilePath,"cam0_distortion_coeffs");
            Mat4x4  mat_imu_cam  = Mat44FromYaml(configFilePath,"T_imu_cam0");
            cout << "d435i camera" << endl;
            cout << "image_width :" << image_width << endl;
            cout << "image_height:" << image_height << endl;
            cout << "cameraMatrix:" << endl << cam0_cameraMatrix << endl;
            cout << "distCoeffs  :" << endl << cam0_distCoeffs << endl;
            cout << "Mat_imu_cam0 :" << endl << mat_imu_cam << endl;
            SE3 T_i_c = SE3(mat_imu_cam.topLeftCorner(3,3),mat_imu_cam.topRightCorner(3,1));
            cam_tracker->init(image_width,image_height,cam0_cameraMatrix,cam0_distCoeffs,T_i_c);
            img0_sub.subscribe(nh, "/vo/image", 1);
            img1_sub.subscribe(nh, "/vo/depth_image", 1);
        }
        if(cam_type==STEREO_EuRoC_MAV)
        {
            int image_width  = getIntVariableFromYaml(configFilePath,"image_width");
            int image_height = getIntVariableFromYaml(configFilePath,"image_height");
            cv::Mat cam0_cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath,"cam0_intrinsics");
            cv::Mat cam0_distCoeffs   = distCoeffsFromYaml(configFilePath,"cam0_distortion_coeffs");
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

            cout << "euroc mav dataset" << endl;
            cout << "image_width :" << image_width << endl;
            cout << "image_height:" << image_height << endl;
            cout << "cam0_cameraMatrix:" << endl << cam0_cameraMatrix << endl;
            cout << "cam0_distCoeffs  :" << endl << cam0_distCoeffs << endl;
            cout << "Mat_camimu_cam0 :" << endl << mat_mavimu_cam0 << endl;
            cout << "cam1_cameraMatrix:" << endl << cam1_cameraMatrix << endl;
            cout << "cam1_distCoeffs  :" << endl << cam1_distCoeffs << endl;
            cout << "Mat_camimu_cam1 :" << endl << mat_mavimu_cam1 << endl;
            cout << "T_c0_c1_rot: " << endl << T_c0_c1.rotation_matrix() << endl;
            cout << "T_c0_c1_tran: " << endl << T_c0_c1.translation() << endl;
            cam_tracker->init(image_width,image_height,
                              cam0_cameraMatrix,cam0_distCoeffs,
                              T_i_c0,
                              STEREO_EuRoC_MAV,
                              1.0,
                              cam1_cameraMatrix,cam1_distCoeffs,
                              T_c0_c1);
            img0_sub.subscribe(nh, "/vo/image0", 1);
            img1_sub.subscribe(nh, "/vo/image1", 1);
        }

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

        cout << "Tracking start!" << endl;
    }

    void imu_callback(const sensor_msgs::ImuConstPtr& msg)
    {
        //SETP1: TO ENU Frame
        Vec3 acc,gyro;
        ros::Time tstamp = msg->header.stamp;
        if(cam_type==DEPTH_D435I)
        {
            acc = Vec3(-msg->linear_acceleration.z,
                       msg->linear_acceleration.x,
                       msg->linear_acceleration.y);
            gyro = Vec3(msg->angular_velocity.z,
                        -msg->angular_velocity.x,
                        -msg->angular_velocity.y);
        }
        if(cam_type==STEREO_EuRoC_MAV)
        {
            gyro = Vec3(msg->angular_velocity.z,
                        -msg->angular_velocity.y,
                        msg->angular_velocity.x);
            acc = Vec3(-msg->linear_acceleration.z,
                       msg->linear_acceleration.y,
                       -msg->linear_acceleration.x);
        }
//        std::cout << std::fixed;
//        std::cout << std::setprecision(2);
//        cout << "gx" << gyro[0] << " gy" << gyro[1] << " gz" << gyro[2] << endl;
//        cout << "ax" << acc[0]  << " ay" << acc[1]  << " az" << acc[2]  << endl;
        Quaterniond q_w_i;
        Vec3        pos_w_i, vel_w_i;
//        this->cam_tracker->imu_feed(tstamp.toSec(),acc,gyro,
//                                    q_w_i,pos_w_i,vel_w_i);
//        this->pose_imu_pub->pubPose(q_w_i,pos_w_i,tstamp);
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
        //input
        if(cam_type==DEPTH_D435I)
        {
            cv_bridge::CvImagePtr cvbridge_image  = cv_bridge::toCvCopy(img0_Ptr, img0_Ptr->encoding);
            cv_bridge::CvImagePtr cvbridge_depth_image  = cv_bridge::toCvCopy(img1_Ptr, img1_Ptr->encoding);
            bool newkf;//new key frame
            bool reset_cmd;//reset command to localmap node
            this->cam_tracker->image_feed(tstamp.toSec(),
                                          cvbridge_image->image,
                                          cvbridge_depth_image->image,
                                          newkf,
                                          reset_cmd);
            if(newkf) kf_pub->pub(*cam_tracker->curr_frame,tstamp);
            if(reset_cmd) kf_pub->cmdLMResetPub(ros::Time(tstamp));
            frame_pub->pubFramePtsPoseT_c_w(this->cam_tracker->curr_frame->getValid3dPts(),
                                            this->cam_tracker->curr_frame->T_c_w,
                                            tstamp);
            path_pub->pubPathT_c_w(this->cam_tracker->curr_frame->T_c_w,tstamp);
            SE3 T_map_c =SE3();
            try{
                listenerOdomMap.lookupTransform("map","odom",ros::Time(0), tranOdomMap);
                tf::Vector3 tf_t= tranOdomMap.getOrigin();
                tf::Quaternion tf_q = tranOdomMap.getRotation();
                SE3 T_map_odom(Quaterniond(tf_q.w(),tf_q.x(),tf_q.y(),tf_q.z()),
                               Vec3(tf_t.x(),tf_t.y(),tf_t.z()));
                T_map_c = T_map_odom.inverse()*this->cam_tracker->curr_frame->T_c_w.inverse();
                path_lc_pub->pubPathT_w_c(T_map_c,tstamp);
            }
            catch (tf::TransformException ex)
            {
                //cout<<"no transform between map and odom yet."<<endl;
            }
            //            drawKeyPts(img_vis, vVec2_2_vcvP2f(pts2d));
            //            vector<Vec2> outlier;
            //            outlier.insert(outlier.end(), outlier_tracking.begin(), outlier_tracking.end());
            //            outlier.insert(outlier.end(), outlier_reproject.begin(), outlier_reproject.end());
            //            //drawOutlier(img_vis,outlier);
            //            drawFlow(img_vis,lm2d_from,lm2d_to);
            cvtColor(cvbridge_image->image,img_vis,CV_GRAY2BGR);
            drawFrame(img_vis,*this->cam_tracker->curr_frame);
            visualizeDepthImg(dimg_vis,*this->cam_tracker->curr_frame);
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_vis).toImageMsg();
            sensor_msgs::ImagePtr dimg_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dimg_vis).toImageMsg();
            img_pub.publish(img_msg);
            dimg_pub.publish(dimg_msg);
        }
        if(cam_type==STEREO_EuRoC_MAV)
        {
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
            path_pub->pubPathT_c_w(this->cam_tracker->curr_frame->T_c_w,tstamp);
            SE3 T_map_c =SE3();
            try{
                listenerOdomMap.lookupTransform("map","odom",ros::Time(0), tranOdomMap);
                tf::Vector3 tf_t= tranOdomMap.getOrigin();
                tf::Quaternion tf_q = tranOdomMap.getRotation();
                SE3 T_map_odom(Quaterniond(tf_q.w(),tf_q.x(),tf_q.y(),tf_q.z()),
                               Vec3(tf_t.x(),tf_t.y(),tf_t.z()));
                T_map_c = T_map_odom.inverse()*this->cam_tracker->curr_frame->T_c_w.inverse();
                path_lc_pub->pubPathT_w_c(T_map_c,tstamp);
            }
            catch (tf::TransformException ex)
            {
                //cout<<"no transform between map and odom yet."<<endl;
            }
            cvtColor(cam_tracker->curr_frame->img0,img_vis,CV_GRAY2BGR);
            drawFrame(img_vis,*this->cam_tracker->curr_frame);
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_vis).toImageMsg();
            img_pub.publish(img_msg);
            cvtColor(cam_tracker->curr_frame->img1,dimg_vis,CV_GRAY2BGR);
            sensor_msgs::ImagePtr dimg_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dimg_vis).toImageMsg();
            img_pub.publish(img_msg);
            dimg_pub.publish(dimg_msg);


        }
        //tt_cb.toc();
    }//image_input_callback(const sensor_msgs::ImageConstPtr & imgPtr, const sensor_msgs::ImageConstPtr & depthImgPtr)
};//class TrackingNodeletClass
}//namespace flvis_ns

PLUGINLIB_EXPORT_CLASS(flvis_ns::TrackingNodeletClass, nodelet::Nodelet)



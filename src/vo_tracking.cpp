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

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <include/tic_toc_ros.h>
#include <include/common.h>
#include <include/depth_camera.h>
#include <include/feature_dem.h>
#include <include/rviz_frame.h>
#include <include/rviz_path.h>
#include <include/camera_frame.h>
#include <include/yamlRead.h>
#include <include/cv_draw.h>
#include <vo_nodelet/KeyFrame.h>
#include <vo_nodelet/CorrectionInf.h>
#include <include/keyframe_msg.h>
#include <include/bundle_adjustment.h>
#include <include/correction_inf_msg.h>
#include <include/octomap_feeder.h>
#include <include/f2f_tracking.h>
#include <include/vi_motion.h>
#include <tf/transform_listener.h>

enum TRACKINGSTATE{UnInit, Working, trackingFail};
struct ID_POSE {
    int    frame_id;
    SE3    T_c_w;
};
namespace vo_nodelet_ns
{
class VOTrackingNodeletClass : public nodelet::Nodelet
{
public:
    VOTrackingNodeletClass()  {;}
    ~VOTrackingNodeletClass() {;}
private:
    //Subscribers
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MyExactSyncPolicy;
    message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
    ros::Subscriber imu_sub;
    ros::Subscriber correction_inf_sub;
    //Tools
    FeatureDEM         *featureDEM;
    F2FTracking        *tracker;
    //VIMOTION
    bool               has_imu;
    VIMOTION           *vimotion;
    //State
    enum TRACKINGSTATE   vo_tracking_state;
    //F2F
    int frameCount;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    int image_width,image_height;
    CameraFrame::Ptr curr_frame,last_frame;
    //KF, Pose Records and Correction Information
    SE3 T_c_w_last_keyframe;
    SE3 T_map_c =SE3();
    KeyFrameMsg* kf_pub;
    deque<ID_POSE> pose_records;
    bool has_feedback;
    CorrectionInfStruct correction_inf;
    //Octomap
    OctomapFeeder* octomap_pub;
    //Visualization
    cv::Mat img_vis;
    cv::Mat dimg_vis;
    image_transport::Publisher img_pub;
    image_transport::Publisher dimg_pub;

    ros::Publisher imu2vision_pose_pub;
    ros::Publisher vision_pose_pub;

    RVIZFrame* frame_pub;
    RVIZPath*  path_pub;
    RVIZPath* path_lc_pub;

    tf::StampedTransform tranOdomMap;
    tf::TransformListener listenerOdomMap;

    virtual void onInit()
    {
        ros::NodeHandle& nh = getMTPrivateNodeHandle();
        //cv::startWindowThread(); //Bug report https://github.com/ros-perception/image_pipeline/issues/201

        imu2vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("imu", 10);
        vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("vision",10);

        //Load Parameter
        string configFilePath;
        nh.getParam("/yamlconfigfile",   configFilePath);
        image_width  = getIntVariableFromYaml(configFilePath,"image_width");
        image_height = getIntVariableFromYaml(configFilePath,"image_height");
        cameraMatrix = cameraMatrixFromYamlIntrinsics(configFilePath);
        distCoeffs = distCoeffsFromYaml(configFilePath);
        double fx = cameraMatrix.at<double>(0,0);
        double fy = cameraMatrix.at<double>(1,1);
        double cx = cameraMatrix.at<double>(0,2);
        double cy = cameraMatrix.at<double>(1,2);
        cout << "cameraMatrix:" << endl << cameraMatrix << endl
             << "distCoeffs:" << endl << distCoeffs << endl
             << "image_width: "  << image_width << " image_height: "  << image_height << endl
             << "fx: "  << fx << " fy: "  << fy <<  " cx: "  << cx <<  " cy: "  << cy << endl;

        featureDEM  = new FeatureDEM(image_width,image_height,5);
        tracker     = new F2FTracking(image_width,image_height);

        //This setting is for realsense d435i. IMU in ENU frame facing forward.
        Mat3x3 R_i_c;
        // 0  0  1
        //-1  0  0
        // 0 -1  0
        R_i_c << 0, 0, 1, -1, 0, 0, 0,-1, 0;
        Vec3   t_i_c=Vec3(0,0,0);
        SE3    T_i_c(R_i_c,t_i_c);
        vimotion    = new VIMOTION(T_i_c);
        has_imu     = false;

        curr_frame = std::make_shared<CameraFrame>();
        last_frame = std::make_shared<CameraFrame>();
        curr_frame->height = last_frame->height = image_height;
        curr_frame->width = last_frame->width = image_width;
        curr_frame->d_camera = last_frame->d_camera = DepthCamera(fx,fy,cx,cy,1000.0);

        frameCount = 0;
        vo_tracking_state = UnInit;
        has_feedback = false;

        //Publish
        path_pub  = new RVIZPath(nh,"/vo_path");
        path_lc_pub  = new RVIZPath(nh,"/vo_path_lc");
        frame_pub = new RVIZFrame(nh,"/vo_camera_pose","/vo_curr_frame");
        kf_pub    = new KeyFrameMsg(nh,"/vo_kf");
        octomap_pub = new OctomapFeeder(nh,"/vo_octo_tracking","vo_local",1);
        octomap_pub->d_camera=curr_frame->d_camera;
        image_transport::ImageTransport it(nh);
        img_pub = it.advertise("/vo_img", 1);
        dimg_pub = it.advertise("/vo_dimg", 1);
        //Subscribe
        //Sync Subscribe Image and Rectified Depth Image
        image_sub.subscribe(nh, "/vo/image", 1);
        depth_sub.subscribe(nh, "/vo/depth_image", 1);
        exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(2), image_sub, depth_sub);
        exactSync_->registerCallback(boost::bind(&VOTrackingNodeletClass::image_input_callback, this, _1, _2));
        //Correction information
        correction_inf_sub = nh.subscribe<vo_nodelet::CorrectionInf>(
                    "/vo_localmap_feedback",
                    1,
                    boost::bind(&VOTrackingNodeletClass::correction_feedback_callback, this, _1));
        imu_sub = nh.subscribe<sensor_msgs::Imu>(
                    "/imu",
                    10,
                    boost::bind(&VOTrackingNodeletClass::imu_callback, this, _1));
        cout << "init Subscribe" << endl;

        cout << "onInit() finished" << endl;
    }

#define USE_RSD435I (1)

    void imu_callback(const sensor_msgs::ImuConstPtr& msg)
    {
        if(!has_imu){
            has_imu = true;
        }
        //SETP1: TO ENU Frame
        Vec3 acc,gyro;
        double stamp = msg->header.stamp.toSec();
        if(USE_RSD435I)
        {
            acc = Vec3(-msg->linear_acceleration.z,
                       msg->linear_acceleration.x,
                       msg->linear_acceleration.y);
            gyro = Vec3(msg->angular_velocity.z,
                        -msg->angular_velocity.x,
                        -msg->angular_velocity.y);

        }else//Default
        {
            gyro = Vec3(msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z);
            acc = Vec3(msg->linear_acceleration.x,
                       msg->linear_acceleration.y,
                       msg->linear_acceleration.z);
        }
        if(!(vimotion->imu_initialized))
        {
            //estimate orientation
            vimotion->viIMUinitialization(IMUSTATE(stamp,acc,gyro));
        }else {
            //estimate pos vel and orientation
            vimotion->viIMUPropagation(IMUSTATE(stamp,acc,gyro));
            geometry_msgs::PoseStamped pose;
            SE3 T_w_i_from_imu;
            Vec3 vel_from_imu;
            vimotion->viGetLatestImuState(T_w_i_from_imu,vel_from_imu);
            SE3 T_w_c=T_w_i_from_imu*vimotion->T_i_c;
            pose.header.stamp = msg->header.stamp;
            pose.header.frame_id = "map";
            Vec3 p = T_w_c.translation();
            Quaterniond q = T_w_c.unit_quaternion();
            pose.pose.position.x = p[0];
            pose.pose.position.y = p[1];
            pose.pose.position.z = p[2];
            pose.pose.orientation.w = q.w();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            this->imu2vision_pose_pub.publish(pose);
        }
    }

    void correction_feedback_callback(const vo_nodelet::CorrectionInf::ConstPtr& msg)
    {
        //unpacking and update the structure
        CorrectionInfMsg::unpack(msg,
                                 correction_inf.frame_id,
                                 correction_inf.T_c_w,
                                 correction_inf.lm_count,
                                 correction_inf.lm_id,
                                 correction_inf.lm_3d,
                                 correction_inf.lm_outlier_count,
                                 correction_inf.lm_outlier_id);
        has_feedback=true;
    }

    void image_input_callback(const sensor_msgs::ImageConstPtr & imgPtr, const sensor_msgs::ImageConstPtr & depthImgPtr)
    {
        tic_toc_ros tt_cb;

        frameCount++;
        if(frameCount<30) return;

        //15hz Frame Rate
        //if((frameCount%2)==0) return;
        //cout << "Frame No: " << frameCount << endl;
        curr_frame->frame_id = frameCount;
        //Greyscale Img
        curr_frame->clear();
        cv_bridge::CvImagePtr cvbridge_image  = cv_bridge::toCvCopy(imgPtr, imgPtr->encoding);
        curr_frame->img=cvbridge_image->image;
        //equalizeHist(curr_frame->img, curr_frame->img);
        //Depth Img
        cv_bridge::CvImagePtr cvbridge_depth_image  = cv_bridge::toCvCopy(depthImgPtr, depthImgPtr->encoding);
        curr_frame->d_img=cvbridge_depth_image->image;
        curr_frame->frame_time = imgPtr->header.stamp;
        cvtColor(curr_frame->img,img_vis,CV_GRAY2BGR);
        ros::Time currStamp = imgPtr->header.stamp;
        switch(vo_tracking_state)
        {
        case UnInit:
        {
            Mat3x3 R_w_c;
            // 0  0  1
            //-1  0  0
            // 0 -1  0


            R_w_c << 0, 0, 1, -1, 0, 0, 0,-1, 0;
            Vec3   t_w_c=Vec3(0,0,1);
            SE3    T_w_c(R_w_c,t_w_c);

            curr_frame->T_c_w=T_w_c.inverse();//Tcw = (Twc)^-1
            if(this->has_imu)
            {
                if(vimotion->imu_initialized)
                {
                    Eigen::Quaterniond q_init_w_i;
                    vimotion->viVisiontrigger(q_init_w_i);
                    Vec3 rpy = Q2rpy(q_init_w_i);
                    cout << "roll:"   << rpy[0]*57.2958
                         << " pitch:" << rpy[1]*57.2958
                         << " yaw:"   << rpy[2]*57.2958 << endl;
                    Mat3x3 R_w_c = q_init_w_i.toRotationMatrix()*vimotion->T_i_c.rotation_matrix();
                    SE3    T_w_c(R_w_c,t_w_c);
                    curr_frame->T_c_w=T_w_c.inverse();//Tcw = (Twc)^-1
                    cout << "use imu pose" << endl;
                }
                else
                {
                    break;
                }
            }

            vector<Vec2> pts2d;
            vector<Vec3> pts3d_c;
            vector<cv::Mat>  descriptors;
            vector<bool> maskHas3DInf;

            featureDEM->detect(curr_frame->img,pts2d,descriptors);
            cout << "Detect " << pts2d.size() << " Features"<< endl;
            for(size_t i=0; i<pts2d.size(); i++)
            {
                curr_frame->landmarks.push_back(LandMarkInFrame(descriptors.at(i),
                                                                pts2d.at(i),
                                                                Vec3(0,0,0),
                                                                false,
                                                                curr_frame->T_c_w));
            }
            curr_frame->depthInnovation();


            drawKeyPts(img_vis, vVec2_2_vcvP2f(pts2d));
            frame_pub->pubFramePtsPoseT_c_w(curr_frame->getValid3dPts(),curr_frame->T_c_w,currStamp);
            path_pub->pubPathT_c_w(curr_frame->T_c_w,currStamp);

            kf_pub->pub(*curr_frame,currStamp);
            T_c_w_last_keyframe = curr_frame->T_c_w;

            vo_tracking_state = Working;
            cout << "vo_tracking_state = Working" << endl;

            //if has tf between map and odom
            if(1)
            {
                try
                {
                    listenerOdomMap.lookupTransform("map","odom",ros::Time(0), tranOdomMap);


              tf::Vector3 tf_t= tranOdomMap.getOrigin();
              tf::Quaternion tf_q = tranOdomMap.getRotation();
              Vec3 se3_t(tf_t.x(),tf_t.y(),tf_t.z());
              Quaterniond se3_q(tf_q.w(),tf_q.x(),tf_q.y(),tf_q.z());
              //cout<<tf_t.x()<<" "<<tf_t.y()<<" "<<tf_t.z()<<" "<<tf_q.x()<<" "<<tf_q.y()<<" "<<tf_q.z()<<" "<<tf_q.w()<<" "<<endl;
              SE3 T_map_odom(se3_q,se3_t);
              T_map_c = T_map_odom.inverse()*curr_frame->T_c_w.inverse();
              path_lc_pub->pubPathT_w_c(T_map_c,currStamp);



                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("%s",ex.what());
                    cout<<"no transform between map and odom yet."<<endl;
                }
            }




            break;
        }


        case Working:
        {
            /* F2F Workflow
                     STEP1: Recover from LocalMap Feedback msg
                     STEP2: Track Match and Update to curr_frame
                     STEP3: 2D3D-PNP
                     (Option) ->IMU roll pitch compensation
                     STEP4: F2FBA
                     (Option) ->IMU state update from vision
                     STEP5: Redetect
                     STEP6: Update Landmarks(IIR)
                     STEP6: Record Pose
                     STEP7: Visualize and Publish
                     STEP8: Switch KeyFrame
            */
            //STEP1:
            if(has_feedback)
            {
                //find pose;
                int corr_id = correction_inf.frame_id;
                int old_pose_idx = 0;
                for(int i=(pose_records.size()-1); i>=0; i--)
                {
                    if(pose_records.at(i).frame_id==corr_id)
                    {
                        old_pose_idx=i;
                        break;
                    }
                }
                SE3 old_T_c_w = pose_records.at(old_pose_idx).T_c_w;
                SE3 old_T_c_w_inv = old_T_c_w.inverse();
                SE3 update_T_c_w = correction_inf.T_c_w;
                //update pose records
                for(int i=old_pose_idx; i<pose_records.size(); i++)
                {
                    SE3 T_diff= pose_records.at(i).T_c_w * old_T_c_w_inv;
                    pose_records.at(i).T_c_w = T_diff * update_T_c_w;
                }
                SE3 T_diff= last_frame->T_c_w * old_T_c_w_inv;
                last_frame->T_c_w = T_diff * update_T_c_w;
                last_frame->correctLMP3DWByLMP3DCandT();
                //update last_frame landmake lm_3d_w and mask outlier
                last_frame->forceCorrectLM3DW(correction_inf.lm_count,correction_inf.lm_id,correction_inf.lm_3d);
                last_frame->forceMarkOutlier(correction_inf.lm_outlier_count,correction_inf.lm_outlier_id);
                has_feedback = false;
            }

            //STEP2:
            vector<Vec2> lm2d_from,lm2d_to,outlier_tracking;
            tracker->tracking(*last_frame,
                              *curr_frame,
                              lm2d_from,
                              lm2d_to,
                              outlier_tracking);

            //STEP3:
            vector<cv::Point2f> p2d;
            vector<cv::Point3f> p3d;
            curr_frame->getValid2d3dPair_cvPf(p2d,p3d);
            if(p2d.size()<=10)
            {
                cout << "[Critical Warning] Tracking Fail-no enough lm pairs" << endl;
                break;
            }
            cv::Mat r_ = cv::Mat::zeros(3, 1, CV_64FC1);
            cv::Mat t_ = cv::Mat::zeros(3, 1, CV_64FC1);
            SE3_to_rvec_tvec(last_frame->T_c_w, r_ , t_ );
            cv::Mat inliers;
            solvePnPRansac(p3d,p2d,cameraMatrix,distCoeffs,r_,t_,false,100,2.0,0.99,inliers,cv::SOLVEPNP_P3P);
            curr_frame->T_c_w = SE3_from_rvec_tvec(r_,t_);
            std::vector<uchar> status;
            for (int i = 0; i < (int)p2d.size(); i++)
                status.push_back(0);
            for( int i = 0; i < inliers.rows; i++)
            {
                int n = inliers.at<int>(i);
                status[n] = 1;
            }
            curr_frame->updateLMState(status);

            //(Option) ->IMU roll pitch compensation
            if(this->has_imu)
            {
                vimotion->viVisionRPCompensation(curr_frame->frame_time.toSec(),
                                                 curr_frame->T_c_w,
                                                 0.01);
            }

            //STEP4:
            bundleAdjustment::BAInFrame(*curr_frame);
            vector<Vec2> outlier_reproject;
            double mean_reprojection_error;
            curr_frame->CalReprjInlierOutlier(mean_reprojection_error,outlier_reproject,1.5);
            curr_frame->reprojection_error=mean_reprojection_error;


            //(Option) ->Vision Feedback and bias estimation
            if(this->has_imu)
            {
                vimotion->viCorrectionFromVision(curr_frame->frame_time.toSec(),
                                                 curr_frame->T_c_w,Vec3(0,0,0));
            }

            //STEP5:
            vector<Vec2> newKeyPts;
            vector<cv::Mat>  newDescriptor;
            int newPtsCount;
            featureDEM->redetect(curr_frame->img,
                                 curr_frame->get2dPtsVec(),
                                 newKeyPts,newDescriptor,newPtsCount);
            for(size_t i=0; i<newKeyPts.size(); i++)
            {
                curr_frame->landmarks.push_back(LandMarkInFrame(newDescriptor.at(i),
                                                                newKeyPts.at(i),
                                                                Vec3(0,0,0),
                                                                false,
                                                                curr_frame->T_c_w));
            }
            //STEP6:
            curr_frame->depthInnovation();


            //STEP7:
            ID_POSE tmp;
            tmp.frame_id = curr_frame->frame_id;
            tmp.T_c_w = curr_frame->T_c_w;
            pose_records.push_back(tmp);
            if(pose_records.size() >= 1000)
            {
                pose_records.pop_front();
            }

            //STEP8:
            vector<Vec2> outlier;
            outlier.insert(outlier.end(), outlier_tracking.begin(), outlier_tracking.end());
            outlier.insert(outlier.end(), outlier_reproject.begin(), outlier_reproject.end());
            curr_frame->solving_time=tt_cb.dT_s();
            //drawOutlier(img_vis,outlier);
            drawFlow(img_vis,lm2d_from,lm2d_to);
            drawFrame(img_vis,*curr_frame);
            visualizeDepthImg(dimg_vis,*curr_frame);

            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_vis).toImageMsg();
            sensor_msgs::ImagePtr dimg_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dimg_vis).toImageMsg();
            img_pub.publish(img_msg);
            dimg_pub.publish(dimg_msg);
            frame_pub->pubFramePtsPoseT_c_w(curr_frame->getValid3dPts(),curr_frame->T_c_w);
            path_pub->pubPathT_c_w(curr_frame->T_c_w,currStamp);
            //octomap_pub->pub(curr_frame->T_c_w,curr_frame->d_img,currStamp);


            //if has tf between map and odom
            if(1)
            {
                try
                {
                    listenerOdomMap.lookupTransform("map","odom",ros::Time(0), tranOdomMap);


              tf::Vector3 tf_t= tranOdomMap.getOrigin();
              tf::Quaternion tf_q = tranOdomMap.getRotation();
              Vec3 se3_t(tf_t.x(),tf_t.y(),tf_t.z());
              Quaterniond se3_q(tf_q.w(),tf_q.x(),tf_q.y(),tf_q.z());
              //cout<<tf_t.x()<<" "<<tf_t.y()<<" "<<tf_t.z()<<" "<<tf_q.x()<<" "<<tf_q.y()<<" "<<tf_q.z()<<" "<<tf_q.w()<<" "<<endl;
              SE3 T_map_odom(se3_q,se3_t);
              T_map_c = T_map_odom*curr_frame->T_c_w.inverse();
              path_lc_pub->pubPathT_w_c(T_map_c,currStamp);
              octomap_pub->pub(T_map_c.inverse(),curr_frame->d_img,currStamp);


                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("%s",ex.what());
                    cout<<"no trans between map and odom yet."<<endl;
                }
            }



            //STEP9:

            SE3 T_diff_key_curr = T_c_w_last_keyframe*(curr_frame->T_c_w.inverse());
            Vec3 t=T_diff_key_curr.translation();
            Vec3 r=T_diff_key_curr.so3().log();
            double t_norm = fabs(t[0]) + fabs(t[1]) + fabs(t[2]);
            double r_norm = fabs(r[0]) + fabs(r[1]) + fabs(r[2]);
            if(t_norm>=0.15 || r_norm>=0.2)
            {
                kf_pub->pub(*curr_frame,currStamp);
                T_c_w_last_keyframe = curr_frame->T_c_w;
            }

            break;
        }

        case trackingFail:
        {
            vo_tracking_state = UnInit;
            break;
        }

        default:
            cout<<"error"<<endl;
        }//end of state machine
        cv::waitKey(1);
        last_frame.swap(curr_frame);

        //tt_cb.toc();
    }//image_input_callback(const sensor_msgs::ImageConstPtr & imgPtr, const sensor_msgs::ImageConstPtr & depthImgPtr)
};//class VOTrackingNodeletClass
}//namespace vo_nodelet_ns

PLUGINLIB_EXPORT_CLASS(vo_nodelet_ns::VOTrackingNodeletClass, nodelet::Nodelet)



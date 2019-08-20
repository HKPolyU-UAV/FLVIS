#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <utils/tic_toc_ros.h>
#include <include/common.h>
#include <include/depth_camera.h>
#include <include/feature_dem.h>
#include <include/rviz_frame.h>
#include <include/rviz_path.h>
#include <include/camera_frame.h>
#include <include/yamlRead.h>
#include <include/cv_draw.h>

using namespace cv;
using namespace std;



enum TRACKINGSTATE{UnInit, Working, trackingFail};

namespace vo_nodelet_ns
{


class VOTrackingNodeletClass : public nodelet::Nodelet
{
public:
  VOTrackingNodeletClass()  {;}
  ~VOTrackingNodeletClass() {;}

private:

  message_filters::Subscriber<sensor_msgs::Image> image_sub;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MyExactSyncPolicy;
  message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

  //State Machine
  enum TRACKINGSTATE   trackingState;

  //Tools
  FeatureDEM         *featureDEM;

  //F2F
  int frameCount;
  Mat cameraMatrix;
  Mat distCoeffs;
  int image_width,image_height;
  CameraFrame::Ptr currFrame,lastFrame;

  //Visualization
  Mat currShowImg;
  RVIZFrame* framePub;
  RVIZPath*  pathPub;
  image_transport::ImageTransport *it;
  image_transport::Publisher cv_pub;

  virtual void onInit()
  {
    ros::NodeHandle& nh = getPrivateNodeHandle();
    cv::startWindowThread();

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
    cout << "cameraMatrix:" << endl << cameraMatrix << endl;
    cout << "distCoeffs:" << endl << distCoeffs << endl;
    cout << "image_width: "  << image_width << endl;
    cout << "image_height: "  << image_height << endl;
    cout << "fx: "  << fx << endl;
    cout << "fy: "  << fy << endl;
    cout << "cx: "  << cx << endl;
    cout << "cy: "  << cy << endl;

    featureDEM  = new FeatureDEM(image_width,image_height,10);
    cout << "init regionBasedDetector" << endl;

    currFrame = std::make_shared<CameraFrame>();
    lastFrame = std::make_shared<CameraFrame>();
    currFrame->d_camera = lastFrame->d_camera = DepthCamera(fx,fy,cx,cy,1000.0);
    frameCount = 0;
    trackingState = UnInit;

    //Publish
    pathPub  = new RVIZPath(nh,"/vo_path");
    framePub = new RVIZFrame(nh,"/vo_camera_pose","/vo_currFrame");
    it = new image_transport::ImageTransport(nh);
    cv_pub = it->advertise("/vo_img", 1);

    //Subscribe
    //Sync Subscribe Image and Rectified Depth Image
    image_sub.subscribe(nh, "/vo/image", 1);
    depth_sub.subscribe(nh, "/vo/depth_image", 1);
    exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(3), image_sub, depth_sub);
    exactSync_->registerCallback(boost::bind(&VOTrackingNodeletClass::image_input_callback, this, _1, _2));
    cout << "init Subscribe" << endl;

    cout << "onInit() finished" << endl;
  }

  void image_input_callback(const sensor_msgs::ImageConstPtr & imgPtr, const sensor_msgs::ImageConstPtr & depthImgPtr)
  {
    tic_toc_ros tt_cb;

    frameCount++;
    cout << endl << "Frame No: " << frameCount << endl;

    //Greyscale Img
    currFrame->clear();
    cv_bridge::CvImagePtr cvbridge_image  = cv_bridge::toCvCopy(imgPtr, imgPtr->encoding);
    currFrame->img=cvbridge_image->image;
    equalizeHist(currFrame->img, currFrame->img);
    //Depth Img
    cv_bridge::CvImagePtr cvbridge_depth_image  = cv_bridge::toCvCopy(depthImgPtr, depthImgPtr->encoding);
    currFrame->d_img=cvbridge_depth_image->image;

    cvtColor(currFrame->img,currShowImg,CV_GRAY2BGR);
    ros::Time currStamp = imgPtr->header.stamp;

    switch(trackingState)
    {
    case UnInit:
    {
      Mat3x3 R;
      // 0  0  1
      //-1  0  0
      // 0 -1  0
      R << 0, 0, 1, -1, 0, 0, 0,-1, 0;
      Vec3   t=Vec3(0,0,1);
      SE3    T_w_c(R,t);
      currFrame->T_c_w=T_w_c.inverse();//Tcw = (Twc)^-1

      vector<Vec2> pts2d;
      vector<Vec3> pts3d_c;
      vector<Mat>  descriptors;
      vector<unsigned char> maskHas3DInf;

      featureDEM->detect(currFrame->img,pts2d,descriptors);
      cout << "Detect " << pts2d.size() << " Features"<< endl;
      currFrame->d_camera.recover3DPtsFromDepthImg(currFrame->d_img,pts2d,pts3d_c,maskHas3DInf);

      //add landmark
      for(size_t i=0; i<pts2d.size(); i++)
      {
        Vec3 pt3d_w = DepthCamera::camera2worldT_c_w(pts3d_c.at(i),currFrame->T_c_w);
        currFrame->landmarks.push_back(
              LandMarkInFrame(pts2d.at(i),pt3d_w,
                              descriptors.at(i),maskHas3DInf.at(i)));
      }
      drawKeyPts(currShowImg, vVec2_2_vcvP2f(pts2d));
      framePub->pubFramePtsPoseT_c_w(currFrame->getValid3dPts(),currFrame->T_c_w,currStamp);
      pathPub->pubPathT_c_w(currFrame->T_c_w,currStamp);

      trackingState = Working;

      break;
    }


    case Working:
    {
      /* F2F Workflow
         STEP1: Track Match
         STEP2: 2D3D-PNP+F2FBA
         STEP3: Redetect
         STEP4: Update Landmarks(IIR)
      */
      //Remove Lost Landmarks and Creat currFrame
      vector<Vec2> lm2d_from;
      vector<Vec2> lm2d_to;
      vector<Vec2> outlier_tracking;
      lastFrame->trackMatchAndUpdateLMs(currFrame->img,
                                        lm2d_from,
                                        lm2d_to,
                                        outlier_tracking);
      //Update currFrame
      for(size_t i=0; i<lastFrame->landmarks.size(); i++)
      {
        currFrame->landmarks.push_back(lastFrame->landmarks.at(i));
        currFrame->landmarks.at(i).lmPt2d=lm2d_to.at(i);
      }
      drawFlow(currShowImg,lm2d_from,lm2d_to);

      vector<Point2f> p2d;
      vector<Point3f> p3d;
      currFrame->getValid2d3dPair_cvPf(p2d,p3d);
      cv::Mat R_;
      cv::Mat r_;
      cv::Mat t_;

      r_ = cv::Mat::zeros(3, 1, CV_64FC1);
      t_ = cv::Mat::zeros(3, 1, CV_64FC1);
      solvePnPRansac(p3d,p2d,cameraMatrix,distCoeffs,r_,t_,false);

      cv::Rodrigues ( r_, R_ );
      Mat3x3 R=cvMat_to_Mat3x3(R_);
      Vec3   t=cvMat_to_Vec3(t_);

      //bundleAdjustment ( pts_3d, pts_2d, K, R, t );

      //6D Inf Update
      currFrame->T_c_w = SE3(R,t);
      framePub->pubFramePtsPoseT_c_w(currFrame->getValid3dPts(),currFrame->T_c_w);
      pathPub->pubPathT_c_w(currFrame->T_c_w,currStamp);

      //Remove Outliers ||reprojection error|| > MAD of all reprojection error
      vector<Vec2> outlier_reproject;
      currFrame->removeReprojectionOutliers(outlier_reproject,5.0);
      vector<Vec2> outlier;
      outlier.insert(outlier.end(), outlier_tracking.begin(), outlier_tracking.end());
      outlier.insert(outlier.end(), outlier_reproject.begin(), outlier_reproject.end());
      cout << "draw Outlier" << endl;
      drawOutlier(currShowImg,outlier);
      //Refill the keyPoints
      vector<Vec2> newKeyPts;
      vector<Mat>  newDescriptor;
      int newPtsCount;
      featureDEM->redetect(currFrame->img,
                           currFrame->get2dPtsVec(),
                           newKeyPts,newDescriptor,newPtsCount);

      //add landmarks with no position information
      Vec3 pt3d_w(0,0,0);
      for(size_t i=0; i<newKeyPts.size(); i++)
      {
        LandMarkInFrame lm = LandMarkInFrame(newKeyPts.at(i),pt3d_w,newDescriptor.at(i),0);
        currFrame->landmarks.push_back(lm);
        //cout << "step " << i << " newkpssize "<< newKeyPts.size() << endl;
      }
      //      cout << "add lms to currFrame" << endl;
      currFrame->updateDepthMeasurement();

      //Update Depth Measurement
      //      static int count=0;
      //      count++;
      //      if(count==4)
      //      {
      //        trackingState = UnInit;
      //        count =0;
      //      }

      break;
    }

    case trackingFail:
    {
      trackingState = UnInit;
      break;
    }

    default:
      cout<<"error"<<endl;
    }//switch(trackingState)


    drawRegion16(currShowImg);
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", currShowImg).toImageMsg();
    cv_pub.publish(img_msg);
    waitKey(1);

    lastFrame.swap(currFrame);

    tt_cb.toc();
  }//image_input_callback(const sensor_msgs::ImageConstPtr & imgPtr, const sensor_msgs::ImageConstPtr & depthImgPtr)


};//class VOTrackingNodeletClass
}//namespace vo_nodelet_ns

PLUGINLIB_EXPORT_CLASS(vo_nodelet_ns::VOTrackingNodeletClass, nodelet::Nodelet)



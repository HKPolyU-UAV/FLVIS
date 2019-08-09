#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <utils/tic_toc_ros.h>
#include <include/common.h>
#include <include/dCamera.h>
#include <include/featureDEM.h>
#include <include/rvizFrame.h>
#include <include/rvizPath.h>
#include <include/cameraFrame.h>



using namespace cv;
using namespace std;


void drawRegion16(Mat& img)
{
  int divH = floor(img.size().height/4);
  int divW = floor(img.size().width/4);
  int x,y;
  for(int i=1; i<=3; i++)//horizon
  {
    y=i*divH;
    line(img, Point(0,y), Point((img.size().width-1),y), Scalar(255,255,255),1);//horizon
    x=i*divW;
    line(img, Point(x,0), Point(x,(img.size().height-1)), Scalar(255,255,255),1);//vertical
  }
}

void drawKeyPts(Mat& img, const vector<Point2f>& KeyPts)
{
  for(size_t i=0; i<KeyPts.size(); i++)
  {
    Point pt(floor(KeyPts.at(i).x),floor(KeyPts.at(i).y));
    circle(img, pt, 2, Scalar( 255, 0, 0 ), 3);
  }
}

void drawFlow(Mat& img, const vector<Point2f>& from, const vector<Point2f>& to)
{
  if(from.size()==to.size())
  {
    for(size_t i=0; i<from.size(); i++)
    {
      Point pt_from(floor(from.at(i).x),floor(from.at(i).y));
      Point pt_to(floor(to.at(i).x),floor(to.at(i).y));
      circle(img, pt_from, 2, Scalar( 0, 255, 0 ));
      arrowedLine(img, pt_from, pt_to, Scalar( 0,204,204),2);
    }
  }
}

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
  DepthCamera        *camera;

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


  virtual void onInit()
  {
    ros::NodeHandle& nh = getPrivateNodeHandle();
    cv::startWindowThread();
    //Read Parameter

    double fx,fy,cx,cy;
    nh.getParam("/image_width",   image_width);
    nh.getParam("/image_height",  image_height);
    nh.getParam("/fx",            fx);
    nh.getParam("/fy",            fy);
    nh.getParam("/cx",            cx);
    nh.getParam("/cy",            cy);

    cout << "image_width: "  << image_width << endl;
    cout << "image_width: "  << image_height << endl;

    //Init
    cameraMatrix = (Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    distCoeffs   = (Mat1d(5, 1) << 0,0,0,0,0);

    featureDEM  = new FeatureDEM(image_width,image_height,10);
    cout << "init regionBasedDetector" << endl;
    camera    = new DepthCamera(fx,fy,cx,cy,1000.0);
    cout << "init DepthCamera" << endl;

    currFrame = std::make_shared<CameraFrame>();
    lastFrame = std::make_shared<CameraFrame>();
    currFrame->camera = lastFrame->camera = *camera;
    frameCount = 0;
    trackingState = UnInit;


    //Publish
    pathPub  = new RVIZPath(nh,"/vo_path");
    framePub = new RVIZFrame(nh,"/vo_camera_pose","/vo_currFrame");
    cout << "init Publish" << endl;

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
      camera->recover3DPtsFromDepthImg(currFrame->d_img,pts2d,pts3d_c,maskHas3DInf);

      //add landmark
      for(size_t i=0; i<pts2d.size(); i++)
      {
        Vec3 pt3d_w = camera->camera2world(pts3d_c.at(i),currFrame->T_c_w);
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
         STEP1: Track
         STEP2: Compute ORB
         STEP3: Match Check
         STEP4: PNP2D3D
         STEP5: F2FOptimize
         STEP6: Redetect
      */

      //Track
      vector<Point2f> trackedLM_cvP2f;
      vector<float> err;
      vector<unsigned char> mask_tracked;
      TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
      calcOpticalFlowPyrLK(lastFrame->img, currFrame->img,
                           lastFrame->get2dPtsVec_cvP2f(), trackedLM_cvP2f,
                           mask_tracked, err, Size(31,31), 2, criteria);

      //Compute ORB
      vector<unsigned char> mask_hasDescriptor;
      vector<Mat>           trackedLMDescriptors;
      vector<KeyPoint>      trackedLM_cvKP;
      Mat descriptorsMat;
      KeyPoint::convert(trackedLM_cvP2f,trackedLM_cvKP);
      Ptr<DescriptorExtractor> extractor = ORB::create();
      extractor->compute(currFrame->img, trackedLM_cvKP, descriptorsMat);

      for(size_t i=0; i<trackedLM_cvP2f.size(); i++)
      {
        unsigned char hasDescriptor = 0;
        for(size_t j=0; j<trackedLM_cvKP.size(); j++)
        {
          if(trackedLM_cvP2f.at(i).x==trackedLM_cvKP.at(j).pt.x &&
             trackedLM_cvP2f.at(i).y==trackedLM_cvKP.at(j).pt.y)
          {//has ORB descriptor
            trackedLMDescriptors.push_back(descriptorsMat.row(j));
            hasDescriptor = 1;
            break;
          }
        }
        mask_hasDescriptor.push_back(hasDescriptor);
        if(hasDescriptor==0)
        {
          cv::Mat zeroDescriptor(cv::Size(32, 1), CV_8U, Scalar(0));
          trackedLMDescriptors.push_back(zeroDescriptor);
        }
      }
      //Match Check
      vector<unsigned char> mask_matchCheck;
      for(size_t i=0; i<trackedLMDescriptors.size(); i++)
      {
        if(norm(lastFrame->landmarks.at(i).lmDescriptor, trackedLMDescriptors.at(i), NORM_HAMMING) <= 100)
        {
          mask_matchCheck.push_back(1);
        }else
        {
          mask_matchCheck.push_back(0);
        }
      }

      //Remove Lost Landmarks and Creat currFrame
      for(int i=lastFrame->landmarks.size()-1; i>=0; i--)
      {
        if(mask_tracked.at(i)!=1       ||
           mask_hasDescriptor.at(i)!=1 ||
           mask_matchCheck.at(i)!=1)
        {
          trackedLM_cvP2f.erase(trackedLM_cvP2f.begin()+i);
          trackedLM_cvKP.erase(trackedLM_cvKP.begin()+i);
          lastFrame->landmarks.erase(lastFrame->landmarks.begin()+i);
        }
      }

      for(size_t i=0; i<lastFrame->landmarks.size(); i++)
      {
        currFrame->landmarks.push_back(lastFrame->landmarks.at(i));
        Vec2 pt = Vec2(trackedLM_cvP2f.at(i).x,trackedLM_cvP2f.at(i).y);
        currFrame->landmarks.at(i).lmPt2d=pt;
      }
      drawFlow(currShowImg,lastFrame->get2dPtsVec_cvP2f(),trackedLM_cvP2f);

      vector<Point2f> p2d;
      vector<Point3f> p3d;
      cv::Mat r_ = cv::Mat::zeros(3, 1, CV_64FC1);
      cv::Mat t_ = cv::Mat::zeros(3, 1, CV_64FC1);
      currFrame->getValid2d3dPair_cvPf(p2d,p3d);
      solvePnP ( p3d, p2d, cameraMatrix, distCoeffs, r_, t_, false, SOLVEPNP_EPNP);

      Mat R_;
      cv::Rodrigues ( r_, R_ );
      Mat3x3 R=cvMat_to_Mat3x3(R_);
      Vec3   t=cvMat_to_Vec3(t_);
      cout<<"Optimizer"<<endl;

      //bundleAdjustment ( pts_3d, pts_2d, K, R, t );

      //3D Inf Update
      currFrame->T_c_w = SE3(R,t);

      framePub->pubFramePtsPoseT_c_w(currFrame->getValid3dPts(),currFrame->T_c_w);
      pathPub->pubPathT_c_w(currFrame->T_c_w,currStamp);

      //lastKeyPts + flowTrackedPts

      //Refill the keyPoints
      vector<Vec2> newKeyPts;
      vector<Mat>  newDescriptor;
      int newPtsCount;
      featureDEM->redetect(currFrame->img,
                           currFrame->get2dPtsVec(),
                           newKeyPts,newDescriptor,newPtsCount);
      cout << "Add " << newKeyPts.size() << "new KeyPoints" << endl;

      //      currKeyPts = flowTrackedPts;
      //      currORBDescriptor = flowTrackedORBDescriptor;
      //      currKeyPts.insert(currKeyPts.end(), newKeyPts.begin(), newKeyPts.end());
      //      currORBDescriptor.insert(currORBDescriptor.end(), newPtsDescriptor.begin(), newPtsDescriptor.end());

      //      //drawKeyPts(currShowImg, newKeyPts);
      //      drawKeyPts(currShowImg, currKeyPts);

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
    imshow("Image",currShowImg);
    if(1)//Window(500,500)
    {
      static int displaytmp=0;
      if(displaytmp==0)
      {
        displaytmp=1;
        cvMoveWindow( "Image", 1300, 500 );
      }
    }
    waitKey(1);

    lastFrame.swap(currFrame);
    cout << "currFrame -> lastFrame" << endl;

    tt_cb.toc();
  }//image_input_callback(const sensor_msgs::ImageConstPtr & imgPtr, const sensor_msgs::ImageConstPtr & depthImgPtr)




};//class VOTrackingNodeletClass
}//namespace vo_nodelet_ns

PLUGINLIB_EXPORT_CLASS(vo_nodelet_ns::VOTrackingNodeletClass, nodelet::Nodelet)



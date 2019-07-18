#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include <common_def.h>
#include <regionBasedDetector.h>
#include <utils/tic_toc_ros.h>



using namespace cv;
using namespace std;

enum TRACKINGSTATE{UnInit, Working, trackingFail};


namespace vo_nodelet_ns
{

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

void drawKeyPts(Mat& img, vector<Point2f>& KeyPts)
{
  for(size_t i=0; i<KeyPts.size(); i++)
  {
    Point pt(floor(KeyPts.at(i).x),floor(KeyPts.at(i).y));
    circle(img, pt, 2, Scalar( 255, 0, 0 ), 3);
  }
}

void drawFlow(Mat& img, vector<Point2f>& from, vector<Point2f>& to)
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

  enum TRACKINGSTATE trackingState;
  regionBasedDetector* detector;
  int frameCount;
  int image_width,image_height;
  Mat currImg,  lastImg ;
  Mat currDImg ,lastDImg;
  vector<Point2f> currKeyPts, lastKeyPts;
  vector<Mat>     currORBDescriptor, lastORBDescriptor;
  FRAMEFEATURES   currFeatures, lastFeatures;


  vector<KeyPoint> KP_tmp;
  Mat              Descriptor_tmp;
  Mat currShowImg;

  void ORBDescriptorMat2VecMat(const Mat& descriptorMat, vector<Mat>& vecDescriptorMat)
  {
    vecDescriptorMat.clear();
    for(int i=0; i<descriptorMat.size().height;i++)
    {
      vecDescriptorMat.push_back(descriptorMat.row(i));
    }
  }


  virtual void onInit()
  {
    ros::NodeHandle& nh = getPrivateNodeHandle();

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
    detector = new regionBasedDetector(image_width,image_height,10);
    frameCount = 0;
    trackingState = UnInit;
    //Publish

    //Sync Subscribe Image and Rectified Depth Image
    image_sub.subscribe(nh, "/vo/image", 1);
    depth_sub.subscribe(nh, "/vo/depth_image", 1);
    exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(3), image_sub, depth_sub);
    exactSync_->registerCallback(boost::bind(&VOTrackingNodeletClass::image_input_callback, this, _1, _2));
  }

  void image_input_callback(const sensor_msgs::ImageConstPtr & imgPtr, const sensor_msgs::ImageConstPtr & depthImgPtr)
  {
    tic_toc_ros tt_cb;

    frameCount++;
    cv_bridge::CvImagePtr cvbridge_image  = cv_bridge::toCvCopy(imgPtr, imgPtr->encoding);
    currImg=cvbridge_image->image;
    equalizeHist(currImg, currImg);
    cv_bridge::CvImagePtr cvbridge_depth_image  = cv_bridge::toCvCopy(depthImgPtr, depthImgPtr->encoding);
    currDImg=cvbridge_depth_image->image;

    cvtColor(currImg,currShowImg,CV_GRAY2BGR);

    switch(trackingState)
    {
    case UnInit:
    {
      detector->detect(currImg,currKeyPts);
      KeyPoint::convert(currKeyPts,KP_tmp);
      Ptr<DescriptorExtractor> extractor = ORB::create();
      extractor->compute(currImg, KP_tmp, Descriptor_tmp);
      KeyPoint::convert(KP_tmp,currKeyPts);
      ORBDescriptorMat2VecMat(Descriptor_tmp,currORBDescriptor);
      cout << "size of currKeyPts:        " << currKeyPts.size() << endl;
      cout << "size of currORBDescriptor: " << currORBDescriptor.size() << endl;
      trackingState = Working;
      drawKeyPts(currShowImg, currKeyPts);
      break;
    }

    case Working:
    {
      cout << "Frame No: " << frameCount << endl;
      cout << currImg.size() << endl;
      vector<Point2f> flowTrackedPts,flowTrackedPtsWithORBdescriptor;
      vector<uchar> status;
      vector<float> err;
      TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
      calcOpticalFlowPyrLK(lastImg, currImg, lastKeyPts, flowTrackedPts, status, err, Size(40,40), 2, criteria);


      //remove OF failure and outlier edge of the camera
      int outlier_count=0;
      for(int i = (flowTrackedPts.size()-1); i>=0; i--)
      {
        Point pt=flowTrackedPts.at(i);
        if(status.at(i)==0 || pt.x<10 || pt.x<10 || pt.x>(image_width-11) || pt.y>(image_height-11))
        {
          outlier_count++;
          lastKeyPts.erase(lastKeyPts.begin()+i);
          lastORBDescriptor.erase(lastORBDescriptor.begin()+i);
          flowTrackedPts.erase(flowTrackedPts.begin()+i);
        }
      }

      //filter by ORB distace
      //Matched:lastKeyPts,lastORBDescriptor,flowTrackedPts
      //flowTrackedPtsWithORBdescriptor is subset of flowTrackedPts
      vector<Mat>     flowTrackedORBDescriptor;
      KeyPoint::convert(flowTrackedPts,KP_tmp);
      Ptr<DescriptorExtractor> extractor = ORB::create();
      extractor->compute(currImg, KP_tmp, Descriptor_tmp);
      ORBDescriptorMat2VecMat(Descriptor_tmp,flowTrackedORBDescriptor);
      KeyPoint::convert(KP_tmp,flowTrackedPtsWithORBdescriptor);
      for(size_t i=0; i<flowTrackedPtsWithORBdescriptor.size(); )
      {
        if(flowTrackedPtsWithORBdescriptor.at(i).x == flowTrackedPts.at(i).x &&
           flowTrackedPtsWithORBdescriptor.at(i).y == flowTrackedPts.at(i).y)
        {//matched
          i++;
        }
        else
        {//un matched matched
          lastKeyPts.erase(lastKeyPts.begin()+i);
          lastORBDescriptor.erase(lastORBDescriptor.begin()+i);
          flowTrackedPts.erase(flowTrackedPts.begin()+i);
        }
      }
      flowTrackedPts = flowTrackedPtsWithORBdescriptor;

      for(int i=flowTrackedPtsWithORBdescriptor.size()-1; i>=0; i--)
      {
        if(norm(lastORBDescriptor.at(i), flowTrackedORBDescriptor.at(i), NORM_HAMMING) > 20 )
        {
          lastKeyPts.erase(lastKeyPts.begin()+i);
          lastORBDescriptor.erase(lastORBDescriptor.begin()+i);
          flowTrackedPts.erase(flowTrackedPts.begin()+i);
          flowTrackedORBDescriptor.erase(flowTrackedORBDescriptor.begin()+i);
        }
      }
      //EPNP RANSIC + Optimizer to Estimate the Camera Pose
      //lastKeyPts + flowTrackedPts

//      cout << "size of lastKeyPts:               " << lastKeyPts.size() << endl;
//      cout << "size of lastORBDescriptor:        " << lastORBDescriptor.size() << endl;
//      cout << "size of flowTrackedPts:           " << flowTrackedPts.size() << endl;
//      cout << "size of flowTrackedORBDescriptor: " << flowTrackedORBDescriptor.size() << endl;

      //Refill


      //Refill the keyPoints
      vector<Point2f> newKeyPts;
      vector<Mat>     newPtsDescriptor;
      int newPtsCount;
      detector->refill(currImg,flowTrackedPts,newKeyPts,newPtsCount);
      vector<Mat> newORBDescriptor;
      KeyPoint::convert(newKeyPts,KP_tmp);
      extractor->compute(currImg, KP_tmp, Descriptor_tmp);
      ORBDescriptorMat2VecMat(Descriptor_tmp,newPtsDescriptor);
      KeyPoint::convert(KP_tmp,newKeyPts);
      cout << "Add " << newKeyPts.size() << "new KeyPoints" << endl;

      currKeyPts = flowTrackedPts;
      currORBDescriptor = flowTrackedORBDescriptor;
      currKeyPts.insert(currKeyPts.end(), newKeyPts.begin(), newKeyPts.end());
      currORBDescriptor.insert(currORBDescriptor.end(), newPtsDescriptor.begin(), newPtsDescriptor.end());

      //drawKeyPts(currShowImg, newKeyPts);
      drawKeyPts(currShowImg, currKeyPts);
      drawFlow(currShowImg,lastKeyPts,flowTrackedPts);

    }

      break;
    case trackingFail:
    {
      detector->detect(currImg,currKeyPts);
      break;
    }

    default:
      cout<<"error"<<endl;
    }//switch(trackingState)


    drawRegion16(currShowImg);
    imshow("keypoints",currShowImg);
    waitKey(1);

    lastKeyPts=currKeyPts;
    lastORBDescriptor = currORBDescriptor;
    lastImg = currImg;
    lastDImg = currDImg;

    cout << endl;
    tt_cb.toc();
    cout << endl;


  }//image_input_callback(const sensor_msgs::ImageConstPtr & imgPtr, const sensor_msgs::ImageConstPtr & depthImgPtr)

};//class VOTrackingNodeletClass
}//namespace vo_nodelet_ns

PLUGINLIB_EXPORT_CLASS(vo_nodelet_ns::VOTrackingNodeletClass, nodelet::Nodelet)



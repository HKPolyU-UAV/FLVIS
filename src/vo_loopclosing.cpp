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


using namespace cv;
using namespace std;

namespace vo_nodelet_ns
{


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

    void frame_callback(const vo_nodelet::KeyFrameConstPtr& msg)
    {
        KeyFrameStruct kf;
        KeyFrameMsg::unpack(msg,kf.frame_id,kf.img,kf.lm_id,kf.lm_2d,kf.lm_3d,kf.lm_descriptor,kf.T_c_w);

        imshow("loopclosing", kf.img);
        waitKey(1);

        cout << endl << "Local Map BA Callback" << endl;
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

        sub_kf = nh.subscribe<vo_nodelet::KeyFrame>(
                    "/vo_kf",
                    10,
                    boost::bind(&VOLoopClosingNodeletClass::frame_callback, this, _1));

    }




};//class VOLoopClosingNodeletClass
}//namespace vo_nodelet_ns



PLUGINLIB_EXPORT_CLASS(vo_nodelet_ns::VOLoopClosingNodeletClass, nodelet::Nodelet)


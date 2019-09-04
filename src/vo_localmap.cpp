#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vo_nodelet/KeyFrame.h>
#include <geometry_msgs/Vector3.h>
#include <include/keyframe_msg.h>

using namespace cv;
using namespace std;

namespace vo_nodelet_ns
{


class VOLocalMapNodeletClass : public nodelet::Nodelet
{
public:
    VOLocalMapNodeletClass()  {;}
    ~VOLocalMapNodeletClass() {;}

private:
    ros::Subscriber sub_kf;

    void frame_callback(const vo_nodelet::KeyFrameConstPtr& msg)
    {
        cv::Mat img;
        vector<uint64_t> lm_id;
        vector<Vec2> lm_2d;
        vector<Vec3> lm_3d;
        SE3 pose;
        KeyFrameMsg::unpack(msg,img,lm_id,lm_2d,lm_3d,pose);

//        for(size_t i=0; i<lm_id.size(); i++)
//        {
//            cout << lm_id.at(i) << " " << lm_2d.at(i).transpose() << " "  << lm_3d.at(i).transpose() << endl;
//        }
        cout << endl << "Local Map BA Callback" << endl;
    }

    virtual void onInit()
    {
        ros::NodeHandle nh = getNodeHandle();
        sub_kf = nh.subscribe<vo_nodelet::KeyFrame>(
                    "/vo_kf",
                    10,
                    boost::bind(&VOLocalMapNodeletClass::frame_callback, this, _1));
    }




};//class VOLocalMapNodeletClass
}//namespace vo_nodelet_ns


PLUGINLIB_EXPORT_CLASS(vo_nodelet_ns::VOLocalMapNodeletClass, nodelet::Nodelet)


#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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

    virtual void onInit()
    {
        ros::NodeHandle& node = getPrivateNodeHandle();
        //Pub
    }


};//class VOLocalMapNodeletClass
}//namespace vo_nodelet_ns


PLUGINLIB_EXPORT_CLASS(vo_nodelet_ns::VOLocalMapNodeletClass, nodelet::Nodelet)


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "stdio.h"
#include <fstream>

using namespace  std;

ros::Subscriber sub;
bool enable_output_file;
bool enable_repub;
std::ofstream fd;
std::string output_file_path;
std::string sub_topic;
std::string sub_type;
std::string repub_topic;
std::string repub_type;

static void republishtopic(ros::Time stamp, double x, double y, double z,
                           double qw, double qx, double qy, double qz)
{

}


static void process(ros::Time stamp, double x, double y, double z,
               double qw, double qx, double qy, double qz)
{
  if(enable_output_file)
  {
    fd << setprecision(6)
       << stamp << " "
       << setprecision(9)
       << x << " "
       << y << " "
       << z << " "
       << qw << " "
       << qx << " "
       << qy << " "
       << qz << std::endl;
  }
  if(enable_repub)
  {
    republishtopic(stamp,x,y,z,qw,qx,qy,qz);
  }
}





void TransformStamped_callback(const geometry_msgs::TransformStampedConstPtr msg)
{
  process(msg->header.stamp,
          msg->transform.translation.x,
          msg->transform.translation.x,
          msg->transform.translation.x,
          msg->transform.rotation.w,
          msg->transform.rotation.x,
          msg->transform.rotation.y,
          msg->transform.rotation.z);
}

void PointStamped_callback(const geometry_msgs::PointStampedConstPtr msg)
{
  process(msg->header.stamp,
          msg->point.x,
          msg->point.y,
          msg->point.z,
          1, 0, 0, 0);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "repub_rec");
  ros::NodeHandle nh("~");

  nh.getParam("sub_topic",        sub_topic);
  nh.getParam("sub_type",         sub_type);
  nh.getParam("repub_topic",      repub_topic);
  nh.getParam("repub_type",       repub_type);
  nh.getParam("output_file_path", output_file_path);
  cout << output_file_path << endl;
  if(output_file_path=="0")
  {
    enable_output_file = false;
  }else
  {
    enable_output_file = true;
    fd.open(output_file_path.c_str());
  }
  if(repub_type == "0")
  {
    enable_repub = false;
  }
  else
  {
    enable_repub = true;
    if(repub_type == "0")
    {
      enable_repub = false;
    }
  }

  if(sub_type=="TransformStamped")
  {
    sub = nh.subscribe(sub_topic, 10, TransformStamped_callback);
  }
  if(sub_type=="PointStamped")
  {
    sub = nh.subscribe(sub_topic, 10, PointStamped_callback);
  }


  ros::spin();

  return 0;
}

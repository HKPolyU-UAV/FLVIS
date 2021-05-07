#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "stdio.h"
#include <fstream>
#include <signal.h>
#include "include/common.h"


using namespace  std;

ros::Publisher repub;
ros::Subscriber sub;
bool enable_output_file;
bool enable_repub;
std::ofstream fd;
std::string output_file_path;
std::string sub_topic;
std::string sub_type;
std::string repub_topic;
std::string repub_type;
nav_msgs::Path path;
nav_msgs::Path record_path;
int frame_id_path = 0;
int last_pose_n = -2;
int cur_pose_n = -1;


static void republishtopic(ros::Time stamp, double x, double y, double z,
                           double qw, double qx, double qy, double qz)
{
    if(repub_type == "PoseStamped")
    {
        geometry_msgs::PoseStamped latest_pose;
        latest_pose.header.frame_id = "map";
        latest_pose.header.stamp=stamp;
        latest_pose.pose.orientation.w = qw;
        latest_pose.pose.orientation.x = qx;
        latest_pose.pose.orientation.y = qy;
        latest_pose.pose.orientation.z = qz;
        latest_pose.pose.position.x = x;
        latest_pose.pose.position.y = y;
        latest_pose.pose.position.z = z;
        repub.publish(latest_pose);
    }
    if(repub_type == "Path")
    {
        geometry_msgs::PoseStamped latest_pose;
        latest_pose.header.frame_id = "map";
        latest_pose.header.stamp=stamp;
        latest_pose.pose.orientation.w = qw;
        latest_pose.pose.orientation.x = qx;
        latest_pose.pose.orientation.y = qy;
        latest_pose.pose.orientation.z = qz;
        latest_pose.pose.position.x = x;
        latest_pose.pose.position.y = y;
        latest_pose.pose.position.z = z;

        path.header.frame_id = "map";
        path.header.stamp = stamp;
        path.poses.push_back(latest_pose);
        //    if(path.poses.size()>=1000)
        //    {
        //      path.poses.erase(path.poses.begin());
        //    }
        repub.publish(path);
    }
}

static void process(ros::Time stamp, double x, double y, double z,
                    double qw, double qx, double qy, double qz)
{
    static ros::Time last_time=ros::Time::now();
    if((ros::Time::now().toSec()-last_time.toSec())>0.1)
    {
        if(enable_output_file)
        {
            fd << setprecision(6)
               << stamp << " "
               << setprecision(6)
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
}

static void writetokittifile(ros::Time stamp, double x, double y, double z,
                             double qw, double qx, double qy, double qz)
{

    if(enable_output_file)
    {
        Vec3 t(x,y,z);
        Quaterniond q(qw,qx,qy,qz);
        Mat3x3 R_ = q.toRotationMatrix();
        fd << setprecision(6) << R_(0,0) << " " << R_(0,1) << " " << R_(0,2) << " " << t[0] << " ";
        fd << setprecision(6) << R_(1,0) << " " << R_(1,1) << " " << R_(1,2) << " " << t[1] << " ";
        fd << setprecision(6) << R_(2,0) << " " << R_(2,1) << " " << R_(2,2) << " " << t[2] << std::endl;
//        fd << setprecision(6)
//           << stamp << " "
//           << setprecision(9)
//           << x << " "
//           << y << " "
//           << z << " "
//           << qw << " "
//           << qx << " "
//           << qy << " "
//           << qz << std::endl;
    }

}

void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    if(sub_type=="NavPath" && enable_output_file == true)
    {
        cout << "save last navpath to " << output_file_path.c_str() << endl;
        if(record_path.poses.size()>0)
        {
            cout << "record_path.poses.size()>0" << record_path.poses.size() << endl;
            for(size_t i = 0; i< record_path.poses.size(); i++)
            {
                geometry_msgs::PoseStamped pose_msg = record_path.poses[i];
                writetokittifile(pose_msg.header.stamp,
                                 pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
                                 pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z);
            }
            fd.close();
            cout << "close file" << endl;
        }

    }
    ros::shutdown();
}

void TransformStamped_callback(const geometry_msgs::TransformStampedConstPtr msg)
{
    process(msg->header.stamp,
            msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z,
            msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y,  msg->transform.rotation.z);
}

void PointStamped_callback(const geometry_msgs::PointStampedConstPtr msg)
{
    process(msg->header.stamp,
            msg->point.x, msg->point.y, msg->point.z,
            1, 0, 0, 0);
}

void PoseStamped_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
    process(msg->header.stamp,
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void Odometry_callback(const nav_msgs::OdometryConstPtr msg)
{

    process(msg->header.stamp,
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}


void NavPath_callback(const nav_msgs::PathConstPtr msg)
{
    //    static int count = 1;
    //    cout << "record nav count" << count << endl;
    record_path = nav_msgs::Path(*msg);
    //    count ++;
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
        if(repub_type == "PoseStamped")
        {
            repub = nh.advertise<geometry_msgs::PoseStamped>(repub_topic, 2);
        }
        if(repub_type == "Path")
        {
            repub = nh.advertise<nav_msgs::Path>(repub_topic, 2);;
        }
    }

    if(sub_type=="TransformStamped")
    {
        sub = nh.subscribe(sub_topic, 2, TransformStamped_callback);
    }
    if(sub_type=="PointStamped")
    {
        sub = nh.subscribe(sub_topic, 2, PointStamped_callback);
    }
    if(sub_type=="PoseStamped")
    {
        sub = nh.subscribe(sub_topic, 2, PoseStamped_callback);
    }
    if(sub_type=="Odometry")
    {
        sub = nh.subscribe(sub_topic, 2, Odometry_callback);
    }
    if(sub_type=="NavPath")
    {
        sub = nh.subscribe(sub_topic, 2, NavPath_callback);
    }

    signal(SIGINT, mySigintHandler);

    ros::spin();

    return 0;
}

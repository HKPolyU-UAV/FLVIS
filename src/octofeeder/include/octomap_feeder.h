#ifndef OCTOMAP_FEEDER_H
#define OCTOMAP_FEEDER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <include/common.h>
#include <include/depth_camera.h>




class OctomapFeeder
{
    tf::TransformBroadcaster br;
    tf::Transform transform;
    string tf_frame_name;

    ros::Publisher octp_pc_pub;
    //sensor model depth camera

public:
    OctomapFeeder();
    OctomapFeeder(ros::NodeHandle& nh, string pc_topic_name, string tf_frame_name_in, int buffersize=2);
    void pub(const SE3 &T_c_w,const  cv::Mat &d_img, const ros::Time stamp=ros::Time::now());
    DepthCamera  d_camera;
};

#endif // OCTOMAP_FEEDER_H

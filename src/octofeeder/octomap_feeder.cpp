#include "include/octomap_feeder.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include "include/tic_toc_ros.h"


OctomapFeeder::OctomapFeeder()
{

}


OctomapFeeder::OctomapFeeder(ros::NodeHandle& nh, string pc_topic_name, string tf_frame_name_in, int buffersize)
{
    this->tf_frame_name = tf_frame_name_in;
    this->octp_pc_pub = nh.advertise<sensor_msgs::PointCloud2>(pc_topic_name,buffersize);
}

void OctomapFeeder::pub(const SE3 &T_c_w,const  cv::Mat &d_img, const ros::Time stamp)
{
    SE3 T_w_c=T_c_w.inverse();
    Quaterniond q = T_w_c.so3().unit_quaternion();
    Vec3        t = T_w_c.translation();

    this->transform.setOrigin(tf::Vector3(t[0],t[1],t[2]));
    this->transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));

    br.sendTransform(tf::StampedTransform(transform, stamp, "map", this->tf_frame_name));

    PointCloudP::Ptr pc_c(new PointCloudP);
    int width_count=0;
    Vec3 pos_w_c= T_c_w.inverse().translation();
    double height=pos_w_c[2];
    int step = 7;
    int lines = 3;
    for(int v = d_img.rows/2-step*lines-1;v <d_img.rows/2+step*lines;v+=step )
    {
        for(int u = 0; u<d_img.cols; u+=step)
        {
            cv::Point2f pt=cv::Point2f(u,v);
            Vec3 pt3d;
            //CV_16UC1 = Z16 16-Bit unsigned int
            if(isnan(d_img.at<ushort>(pt)))
            {
                continue;
            }
            else
            {
                float z = (d_img.at<ushort>(pt))/d_camera.cam_scale_factor;
                if(z>=0.5&&z<=6.5)
                {
                    Vec3 pt_w = this->d_camera.pixel2worldT_c_w(Vec2(u,v),T_c_w,z);
                    if(pt_w[2]>height+0.3 || pt_w[2]<0)
                    {
                        //continue;
                    }
                    Vec3 pt_c = this->d_camera.pixel2camera(Vec2(u,v),z);
                    PointP p(static_cast<float>(pt_c[0]),
                            static_cast<float>(pt_c[1]),
                            static_cast<float>(pt_c[2]));
                    pc_c->points.push_back(p);
                    width_count++;
                }else
                {
                    continue;
                }
            }
        }
    }
    pc_c->width=width_count;
    pc_c->height=1;
    pc_c->is_dense = false;


    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pc_c,output);
    output.header.frame_id = tf_frame_name;
    octp_pc_pub.publish(output);
    //cout<<"point cloud filter cost: ";

}


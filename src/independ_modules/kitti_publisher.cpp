//SYSTEM HEAD FILE
#include <dirent.h>
#include <stdio.h>
#include <string>
#include <fstream>

//ROS HEAD FILE
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>

//USER HEAD FILE
#include <utils/include/common.h>
#include <visualization/include/rviz_path.h>

using namespace  std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitti_publisher");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher img0_pub = it.advertise("/vo/input_image_0", 1);
    image_transport::Publisher img1_pub = it.advertise("/vo/input_image_1", 1);
    double delay_time_s;
    int    publish_rate_hz;
    bool   publish_gt;
    string dataset_folder_path;
    string img0_folder_path;
    string img1_folder_path;
    string gt_file_path;
    std::ifstream gt_file;
    long gt_file_end;

    nh.getParam("/dataset_pub_delay",   delay_time_s);
    nh.getParam("/dataset_pub_rate",    publish_rate_hz);
    nh.getParam("/dataset_folder_path", dataset_folder_path);
    nh.getParam("/publish_gt",          publish_gt);

    img0_folder_path = img1_folder_path = dataset_folder_path;
    img0_folder_path.append("image_0/");
    img1_folder_path.append("image_1/");

    int len;
    int img_cnt_total = 0;
    struct dirent *pDirent;
    DIR *pDir;
    pDir = opendir(img0_folder_path.c_str());
    if (pDir != NULL) {
        while ((pDirent = readdir(pDir)) != NULL) {
            len = strlen (pDirent->d_name);
            if (len >= 4) {
                if (strcmp (".png", &(pDirent->d_name[len - 4])) == 0) {
                    img_cnt_total++;
                    //printf ("%s\n", pDirent->d_name);
                }
            }
        }
        closedir (pDir);
    }
    cout << "contain " << img_cnt_total << " images" << endl;
    cout << "publish gt ? " << ((publish_gt) ? "yes" : "no") << endl;

    SE3    T_w_c;//pose is based on 1st camera frame (z-axis forward), we need to transfer it to world frame (x-axis forward)

        nh.getParam("/dataset_gt_file", gt_file_path);
        cout << gt_file_path << endl;
        gt_file.open(gt_file_path);
        gt_file.seekg( 0, std::ios::end );
        gt_file_end = gt_file.tellg();
        gt_file.seekg( 0, std::ios::beg );
        Mat3x3 R_w_c;
        // 0  0  1
        //-1  0  0
        // 0 -1  0
        R_w_c << 0, 0, 1, -1, 0, 0, 0,-1, 0;
        Vec3 t_w_c = Vec3(0,0,0);
        T_w_c = SE3(R_w_c,t_w_c);

    ros::Rate rate(publish_rate_hz);

    RVIZPath *path_pub = new RVIZPath(nh,"/gt","map",1,10000);
    ros::Duration(delay_time_s).sleep();

    unsigned int img_cnt=0;
    while(ros::ok())
    {
        if(publish_gt)
        {
            if((gt_file_end-gt_file.tellg())<5)
            {
                break;
            }
            string pose_elements_strings[12];
            double pose_elements[12];
            for(size_t i=0; i<12; i++)
            {
                gt_file >> pose_elements_strings[i];
                pose_elements[i] = std::stod(pose_elements_strings[i]);
            }
            Mat3x4 T;
            T << pose_elements[0], pose_elements[1], pose_elements[2],  pose_elements[3],
                    pose_elements[4], pose_elements[5], pose_elements[6],  pose_elements[7],
                    pose_elements[8], pose_elements[9], pose_elements[10], pose_elements[11];
            SE3 pose(SO3(T.topLeftCorner(3,3)),T.topRightCorner(3,1));
            path_pub->pubPathT_w_c(T_w_c*pose);
        }
        std::stringstream filename_tmp;
        filename_tmp << std::setw(6) << std::setfill('0') << img_cnt;
        std::string filename = filename_tmp.str();
        filename.append(".png");

        string img0_absolute_path = img0_folder_path + filename;
        string img1_absolute_path = img1_folder_path + filename;
//        cout << img0_absolute_path << endl;
//        cout << img1_absolute_path << endl;
        cv::Mat img0 = cv::imread(img0_absolute_path, cv::IMREAD_UNCHANGED);
        cv::Mat img1 = cv::imread(img1_absolute_path, cv::IMREAD_UNCHANGED);
        sensor_msgs::ImagePtr img0_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img0).toImageMsg();
        sensor_msgs::ImagePtr img1_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img1).toImageMsg();
        img0_msg->header.seq= img_cnt;
        img0_msg->header.seq= img_cnt;
        img0_msg->header.stamp = ros::Time::now();
        img1_msg->header = img0_msg->header;
        img0_pub.publish(img0_msg);
        img1_pub.publish(img1_msg);
//        cout << "published" << endl;

        img_cnt++;
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

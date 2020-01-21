#include "include/keyframe_msg.h"

KeyFrameMsg::KeyFrameMsg()
{

}

KeyFrameMsg::KeyFrameMsg(ros::NodeHandle &nh, string topic_name, int buffersize)
{
    kf_pub = nh.advertise<flvis::KeyFrame>(topic_name,1);
}

void KeyFrameMsg::pub(CameraFrame& frame, ros::Time stamp)
{
    flvis::KeyFrame kf;

    kf.header.stamp = stamp;
    kf.frame_id = frame.frame_id;
    cv_bridge::CvImage cvimg(std_msgs::Header(), "mono8", frame.img);
    cvimg.toImageMsg(kf.img);

    cv_bridge::CvImage cv_d_img(std_msgs::Header(), "16UC1", frame.d_img);
    cv_d_img.toImageMsg(kf.d_img);

    vector<int64_t> lm_id;
    vector<Vec2> lm_2d;
    vector<Vec3> lm_3d;
    vector<cv::Mat>  lm_descriptors;

    frame.getKeyFrameInf(lm_id,lm_2d,lm_3d,lm_descriptors);
    kf.lm_count =  static_cast<int32_t>(lm_id.size());

    //    for(size_t i=0; i<lm_id.size(); i++)
    //    {
    //        cout << lm_id.at(i) << " " << lm_2d.at(i).transpose() << " "  << lm_3d.at(i).transpose() << endl;
    //    }

    kf.lm_id_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
    kf.lm_id_data.layout.dim[0].label = "lm_id";
    kf.lm_id_data.layout.dim[0].size = static_cast<uint32_t>(lm_id.size());
    kf.lm_id_data.layout.dim[0].stride = static_cast<uint32_t>(lm_id.size());

    kf.lm_id_data.data.clear();
    kf.lm_id_data.data.insert(kf.lm_id_data.data.end(),lm_id.begin(),lm_id.end());

    kf.lm_descriptor_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
    kf.lm_descriptor_data.layout.dim.push_back(std_msgs::MultiArrayDimension());

    kf.lm_descriptor_data.layout.dim[0].label = "lm_descriptor";
    kf.lm_descriptor_data.layout.dim[0].size = static_cast<uint32_t>(lm_id.size());
    kf.lm_descriptor_data.layout.dim[0].stride = static_cast<uint32_t>(32*lm_id.size());
    kf.lm_descriptor_data.layout.dim[1].label = "32uint_descriptor";
    kf.lm_descriptor_data.layout.dim[1].size = 1;
    kf.lm_descriptor_data.layout.dim[1].stride = 32;


    for(size_t i=0; i<lm_id.size(); i++)
    {
        //cout << "i:" << lm_descriptors.at(i).type() << "  " << "size:" << lm_descriptors.at(i).size << endl;
        //cout << "i " << lm_descriptors.at(i) << endl;
        for(int j=0; j<32; j++)
        {
            kf.lm_descriptor_data.data.push_back(lm_descriptors.at(i).at<uint8_t>(0,j));
            //cout << unsigned(lm_descriptors.at(i).at<uint8_t>(0,j)) << " ";
        }
    }

    for(size_t i=0; i<lm_id.size(); i++)
    {
        Vec2 p2d=lm_2d.at(i);
        Vec3 p3d=lm_3d.at(i);
        geometry_msgs::Vector3 vp2d;
        geometry_msgs::Vector3 vp3d;
        vp2d.x = p2d[0];
        vp2d.y = p2d[1];
        kf.lm_2d_data.push_back(vp2d);
        vp3d.x = p3d[0];
        vp3d.y = p3d[1];
        vp3d.z = p3d[2];
        kf.lm_3d_data.push_back(vp3d);
    }
    //cout << "SE3 T_c_w: " << frame.T_c_w << endl;
    Vec3 t=frame.T_c_w.translation();
    Quaterniond uq= frame.T_c_w.unit_quaternion();
    kf.T_c_w.translation.x=t[0];
    kf.T_c_w.translation.y=t[1];
    kf.T_c_w.translation.z=t[2];
    kf.T_c_w.rotation.w=uq.w();
    kf.T_c_w.rotation.x=uq.x();
    kf.T_c_w.rotation.y=uq.y();
    kf.T_c_w.rotation.z=uq.z();

    kf_pub.publish(kf);
}

void KeyFrameMsg::unpack(flvis::KeyFrameConstPtr kf_const_ptr,
                         int64_t         &frame_id,
                         cv::Mat             &img,
                         cv::Mat             &d_img,
                         int             &lm_count,
                         vector<int64_t> &lm_id,
                         vector<Vec2>    &lm_2d,
                         vector<Vec3>    &lm_3d,
                         vector<cv::Mat>     &lm_descriptors,
                         SE3             &T_c_w,
                         ros::Time       &T)
{
    img.release();
    lm_id.clear();
    lm_2d.clear();
    lm_3d.clear();
    lm_descriptors.clear();

    frame_id = kf_const_ptr->frame_id;
    cv_bridge::CvImagePtr cvbridge_image  = cv_bridge::toCvCopy(kf_const_ptr->img, kf_const_ptr->img.encoding);
    img=cvbridge_image->image;
    cv_bridge::CvImagePtr cvbridge_d_image = cv_bridge::toCvCopy(kf_const_ptr->d_img, kf_const_ptr->d_img.encoding);
    d_img = cvbridge_d_image->image;
    lm_count = kf_const_ptr->lm_count;
    int count =  kf_const_ptr->lm_count;
    for(auto i=0; i<count; i++)
    {
        cv::Mat descriptor = cv::Mat(1,32,CV_8U);
        for(auto j=0; j<32; j++)
        {
            descriptor.at<uint8_t>(0,j)=kf_const_ptr->lm_descriptor_data.data.at(i*32+j);
        }
        lm_descriptors.push_back(descriptor);
        //cout << lm_descriptors.at(i) << endl;
        lm_id.push_back(kf_const_ptr->lm_id_data.data[i]);
        Vec2 p2d(kf_const_ptr->lm_2d_data.at(i).x,kf_const_ptr->lm_2d_data.at(i).y);
        Vec3 p3d(kf_const_ptr->lm_3d_data.at(i).x,kf_const_ptr->lm_3d_data.at(i).y,kf_const_ptr->lm_3d_data.at(i).z);
        lm_2d.push_back(p2d);
        lm_3d.push_back(p3d);
    }
    Vec3 t;
    Quaterniond uq;
    t[0] = kf_const_ptr->T_c_w.translation.x;
    t[1] = kf_const_ptr->T_c_w.translation.y;
    t[2] = kf_const_ptr->T_c_w.translation.z;
    uq.w() = kf_const_ptr->T_c_w.rotation.w;
    uq.x() = kf_const_ptr->T_c_w.rotation.x;
    uq.y() = kf_const_ptr->T_c_w.rotation.y;
    uq.z() = kf_const_ptr->T_c_w.rotation.z;
    T_c_w = SE3(uq,t);
    T = kf_const_ptr->header.stamp;

}

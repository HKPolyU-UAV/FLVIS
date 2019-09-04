#include "keyframe_msg.h"

KeyFrameMsg::KeyFrameMsg()
{

}

KeyFrameMsg::KeyFrameMsg(ros::NodeHandle &nh, string topic_name, int buffersize)
{
    kf_pub = nh.advertise<vo_nodelet::KeyFrame>(topic_name,1);
}

void KeyFrameMsg::pub(CameraFrame& frame, ros::Time stamp)
{
    vo_nodelet::KeyFrame kf;
    kf.header.stamp = stamp;
    cv_bridge::CvImage cvimg(std_msgs::Header(), "mono8", frame.img);
    cvimg.toImageMsg(kf.img);

    vector<uint64_t> lm_id;
    vector<Vec2> lm_2d;
    vector<Vec3> lm_3d;
    frame.getKeyFrameInf(lm_id,lm_2d,lm_3d);
    kf.lm_count = lm_id.size();

//    for(size_t i=0; i<lm_id.size(); i++)
//    {
//        cout << lm_id.at(i) << " " << lm_2d.at(i).transpose() << " "  << lm_3d.at(i).transpose() << endl;
//    }
    cout  << endl;

    kf.lm_id_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
    kf.lm_id_data.layout.dim[0].size = lm_id.size();
    kf.lm_id_data.layout.dim[0].stride = 1;
    kf.lm_id_data.layout.dim[0].label = "iduint64_t";
    kf.lm_id_data.data.clear();
    kf.lm_id_data.data.insert(kf.lm_id_data.data.end(),lm_id.begin(),lm_id.end());

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
    cout << "SE3 T_c_w: " << frame.T_c_w << endl;
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

void KeyFrameMsg::unpack(vo_nodelet::KeyFrameConstPtr kf_const_ptr,
                         Mat &img,
                         vector<uint64_t> &lm_id,
                         vector<Vec2> &lm_2d,
                         vector<Vec3> &lm_3d,
                         SE3 &T_c_w)
{
    img.release();
    lm_id.clear();
    lm_2d.clear();
    lm_3d.clear();
    int count =  kf_const_ptr->lm_count;
    cout << "receive " << count << " lms" << endl;

    for(int i=0; i<count; i++)
    {
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
    cout << "SE3 T_c_w: " << T_c_w << endl;
}

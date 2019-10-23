#include "include/correction_inf_msg.h"

CorrectionInfMsg::CorrectionInfMsg()
{

}

CorrectionInfMsg::CorrectionInfMsg(ros::NodeHandle& nh, string topic_name, int buffersize)
{
    correction_inf_pub = nh.advertise<vo_nodelet::CorrectionInf>(topic_name,1);
}

void CorrectionInfMsg::pub(const int64_t         &frame_id_in,
                           const SE3             &T_c_w_in,
                           const int             &lm_count_in,
                           const vector<int64_t> &lm_id_in,
                           const vector<Vec3>    &lm_3d_in,
                           const int             &lm_outlier_count_in,
                           const vector<int64_t> &lm_outlier_id_in,
                           ros::Time             stamp)
{
    vo_nodelet::CorrectionInf c_inf;
    c_inf.frame_id = frame_id_in;

    Vec3 t=T_c_w_in.translation();
    Quaterniond uq= T_c_w_in.unit_quaternion();
    c_inf.T_c_w.translation.x=t[0];
    c_inf.T_c_w.translation.y=t[1];
    c_inf.T_c_w.translation.z=t[2];
    c_inf.T_c_w.rotation.w=uq.w();
    c_inf.T_c_w.rotation.x=uq.x();
    c_inf.T_c_w.rotation.y=uq.y();
    c_inf.T_c_w.rotation.z=uq.z();

    c_inf.lm_count = lm_count_in;

    c_inf.lm_id_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
    c_inf.lm_id_data.layout.dim[0].label = "lm_id";
    c_inf.lm_id_data.layout.dim[0].size = static_cast<uint32_t>(lm_id_in.size());
    c_inf.lm_id_data.layout.dim[0].stride = static_cast<uint32_t>(lm_id_in.size());
    c_inf.lm_id_data.data.clear();
    c_inf.lm_id_data.data.insert(c_inf.lm_id_data.data.end(),lm_id_in.begin(),lm_id_in.end());

    for(size_t i=0; i<lm_id_in.size(); i++)
    {
        Vec3 p3d=lm_3d_in.at(i);
        geometry_msgs::Vector3 vp3d;
        vp3d.x = p3d[0];
        vp3d.y = p3d[1];
        vp3d.z = p3d[2];
        c_inf.lm_3d_data.push_back(vp3d);
    }

    c_inf.lm_outlier_count = lm_outlier_count_in;

    c_inf.lm_outlier_id_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
    c_inf.lm_outlier_id_data.layout.dim[0].label = "lm_outlier_id";
    c_inf.lm_outlier_id_data.layout.dim[0].size = static_cast<uint32_t>(lm_outlier_id_in.size());
    c_inf.lm_outlier_id_data.layout.dim[0].stride = static_cast<uint32_t>(lm_outlier_id_in.size());
    c_inf.lm_outlier_id_data.data.clear();
    c_inf.lm_outlier_id_data.data.insert(c_inf.lm_outlier_id_data.data.end(),lm_outlier_id_in.begin(),lm_outlier_id_in.end());

    this->correction_inf_pub.publish(c_inf);
}

void CorrectionInfMsg::unpack(vo_nodelet::CorrectionInfConstPtr c_inf_ptr,
                              int64_t &frame_id_out,
                              SE3 &T_c_w_out,
                              int &lm_count_out,
                              vector<int64_t> &lm_id_out,
                              vector<Vec3> &lm_3d_out,
                              int &lm_outlier_count_out,
                              vector<int64_t> &lm_outlier_id_out)
{
    lm_id_out.clear();
    lm_3d_out.clear();
    lm_outlier_id_out.clear();

    frame_id_out = c_inf_ptr->frame_id;

    Vec3 t;
    Quaterniond uq;
    t[0] = c_inf_ptr->T_c_w.translation.x;
    t[1] = c_inf_ptr->T_c_w.translation.y;
    t[2] = c_inf_ptr->T_c_w.translation.z;
    uq.w() = c_inf_ptr->T_c_w.rotation.w;
    uq.x() = c_inf_ptr->T_c_w.rotation.x;
    uq.y() = c_inf_ptr->T_c_w.rotation.y;
    uq.z() = c_inf_ptr->T_c_w.rotation.z;
    T_c_w_out = SE3(uq,t);

    lm_count_out = c_inf_ptr->lm_count;
    for(auto i=0; i<lm_count_out; i++)
    {
        lm_id_out.push_back(c_inf_ptr->lm_id_data.data[i]);
        Vec3 p3d(c_inf_ptr->lm_3d_data.at(i).x,c_inf_ptr->lm_3d_data.at(i).y,c_inf_ptr->lm_3d_data.at(i).z);
        lm_3d_out.push_back(p3d);
    }

    lm_outlier_count_out = c_inf_ptr->lm_outlier_count;
    for(auto i=0; i<lm_outlier_count_out; i++)
    {
        lm_outlier_id_out.push_back(c_inf_ptr->lm_outlier_id_data.data[i]);
    }

}


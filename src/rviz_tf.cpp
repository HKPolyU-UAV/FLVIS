#include "rviz_tf.h"


RVIZTF::RVIZTF()
{

}

void RVIZTF::pubTFT_c_w(const SE3 T_c_w, const ros::Time stamp)
{
    SE3 T_w_c=T_c_w.inverse();
    Quaterniond q = T_w_c.so3().unit_quaternion();
    Vec3        t = T_w_c.translation();

    this->transform.setOrigin(tf::Vector3(t[0],t[1],t[2]));
    this->transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));

    br.sendTransform(tf::StampedTransform(transform, stamp, "world", "vo_local"));
}

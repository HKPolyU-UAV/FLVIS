#include "include/rviz_frame.h"


RVIZFrame::RVIZFrame(ros::NodeHandle& nh,
                     string poseTopicName, string ptsTopicName,
                     int bufferPose, int bufferPts)
{
  pose_pub   = nh.advertise<geometry_msgs::PoseStamped>(poseTopicName, bufferPose);
  marker_pub = nh.advertise<visualization_msgs::Marker>(ptsTopicName, bufferPts);
}

RVIZFrame::~RVIZFrame(){}

//Trsnformation world to camera [R|t]
void RVIZFrame::pubFramePoseT_c_w(const SE3 T_c_w,
                                  const ros::Time stamp)
{
  pubFramePoseT_w_c(T_c_w.inverse(),stamp);
}

//Trsnformation camera to world
void RVIZFrame::pubFramePoseT_w_c(const SE3 T_w_c,
                                  const ros::Time stamp)
{
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id="/world";
  poseStamped.header.stamp = stamp;

  Quaterniond q = T_w_c.so3().unit_quaternion();
  Vec3        t = T_w_c.translation();

  poseStamped.pose.orientation.w = q.w();
  poseStamped.pose.orientation.x = q.x();
  poseStamped.pose.orientation.y = q.y();
  poseStamped.pose.orientation.z = q.z();
  poseStamped.pose.position.x = t[0];
  poseStamped.pose.position.y = t[1];
  poseStamped.pose.position.z = t[2];

  pose_pub.publish(poseStamped);
}

void RVIZFrame::pubFramePtsPoseT_c_w(const vector<Vec3> &pts3d,
                                     const SE3 T_c_w,
                                     const ros::Time stamp)
{
  pubFramePtsPoseT_w_c(pts3d,T_c_w.inverse(),stamp);
}

void RVIZFrame::pubFramePtsPoseT_w_c(const vector<Vec3>& pts3d,
                                     const SE3 T_w_c,
                                     const ros::Time stamp)
{
  pubFramePoseT_w_c(T_w_c);

  visualization_msgs::Marker points, line_list,camera_pyramid;
  points.header.frame_id = line_list.header.frame_id = camera_pyramid.header.frame_id= "/world";
  points.header.stamp = line_list.header.stamp = camera_pyramid.header.stamp = stamp;
  points.ns = line_list.ns = camera_pyramid.ns = "points_and_lines";
  points.action = line_list.action = camera_pyramid.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w =  line_list.pose.orientation.w = camera_pyramid.pose.orientation.w = 1.0;

  points.id = 0;
  line_list.id = 1;
  camera_pyramid.id=2;

  points.type = visualization_msgs::Marker::POINTS;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  camera_pyramid.type = visualization_msgs::Marker::LINE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  // Points are blue
  points.scale.x = 0.05;
  points.scale.y = 0.05;

  points.color.b = 0.7f;
  points.color.a = 1.0;

  // LINE_LIST markers use only the x component of scale, for the line width
  // Line list is greed
  line_list.color.g = 0.8f;
  line_list.color.a = 1.0;
  line_list.scale.x = 0.005;

  // LINE_LIST markers use only the x component of scale, for the line width
  // Line list is greed
  camera_pyramid.color.r = 0.3f;
  camera_pyramid.color.b = 0.5f;
  camera_pyramid.color.a = 1.0;
  camera_pyramid.scale.x = 0.02;

  Vec3 t = T_w_c.translation();
  geometry_msgs::Point camera_pos;
  camera_pos.x=t[0];
  camera_pos.y=t[1];
  camera_pos.z=t[2];

  Vec3 pyramidPoint_c[4];//camera frame
  pyramidPoint_c[0] = Vec3(0.08,0.05,0.06);
  pyramidPoint_c[1] = Vec3(0.08,-0.05,0.06);
  pyramidPoint_c[2] = Vec3(-0.06,-0.05,0.06);
  pyramidPoint_c[3] = Vec3(-0.06,0.05,0.06);

  Vec3 pyramidPoint_w;
  geometry_msgs::Point pyramidPointDraw[4];
  for(int i=0; i<4; i++)
  {
    pyramidPoint_w = T_w_c * pyramidPoint_c[i];
    pyramidPointDraw[i].x=pyramidPoint_w[0];
    pyramidPointDraw[i].y=pyramidPoint_w[1];
    pyramidPointDraw[i].z=pyramidPoint_w[2];
    camera_pyramid.points.push_back(camera_pos);
    camera_pyramid.points.push_back(pyramidPointDraw[i]);
  }
  camera_pyramid.points.push_back(pyramidPointDraw[0]);
  camera_pyramid.points.push_back(pyramidPointDraw[1]);
  camera_pyramid.points.push_back(pyramidPointDraw[1]);
  camera_pyramid.points.push_back(pyramidPointDraw[2]);
  camera_pyramid.points.push_back(pyramidPointDraw[2]);
  camera_pyramid.points.push_back(pyramidPointDraw[3]);
  camera_pyramid.points.push_back(pyramidPointDraw[3]);
  camera_pyramid.points.push_back(pyramidPointDraw[0]);


  // Create the vertices for the points and lines
  for (size_t i = 0; i <pts3d.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = pts3d.at(i)[0];
    p.y = pts3d.at(i)[1];
    p.z = pts3d.at(i)[2];
    points.points.push_back(p);
    line_list.points.push_back(camera_pos);
    line_list.points.push_back(p);
  }

  marker_pub.publish(points);
  marker_pub.publish(line_list);
  marker_pub.publish(camera_pyramid);
}

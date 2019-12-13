// need to be removed.







//    pcl::PointCloud<PointRGB>::Ptr generatePointCloudRGB(Mat& depth, Mat& rgb)
//    {
//      double fx = cameraMatrix.at<double>(0,0);
//      double fy = cameraMatrix.at<double>(1,1);
//      double cx = cameraMatrix.at<double>(0,2);
//      double cy = cameraMatrix.at<double>(1,2);
//      PointCloudRGB::Ptr tmp(new PointCloudRGB());
//      for(int m =0; m<depth.rows; m+=5)
//      {
//        for(int n = 0; n<depth.cols; n+=5)
//        {
//          float d = depth.ptr<float>(m)[n];
//          if(d<0.01 || d>6)
//            continue;
//          pcl::PointXYZRGB p;
//          p.z = d;
//          p.x = (n-cx)*d/fx;
//          p.y = (n-cy)*d/fy;
//          p.r = rgb.ptr<uchar>(m)[n*3];
//          p.g = rgb.ptr<uchar>(m)[n*3+1];
//          p.b = rgb.ptr<uchar>(m)[n*3+2];

//          tmp->points.push_back(p);

//        }
//      }
//      tmp->is_dense = false;
//      return tmp;
//    }
//    pcl::PointCloud<PointI>::Ptr generatePointCloudGrey(Mat& depth, Mat& grey)
//    {
//      double fx = cameraMatrix.at<double>(0,0);
//      double fy = cameraMatrix.at<double>(1,1);
//      double cx = cameraMatrix.at<double>(0,2);
//      double cy = cameraMatrix.at<double>(1,2);
//      PointCloudI::Ptr tmp(new PointCloudI());
//      for(int m =0; m<depth.rows; m+=5)
//      {
//        for(int n = 0; n<depth.cols; n+=5)
//        {
//          float d = depth.ptr<float>(m)[n];
//          if(d<0.01 || d>6)
//            continue;
//          pcl::PointXYZI p;
//          p.z = d;
//          p.x = (n-cx)*d/fx;
//          p.y = (n-cy)*d/fy;
//          p.intensity = static_cast<float>(grey.ptr<uchar>(m)[n]);
//          tmp->points.push_back(p);

//        }
//      }
//      tmp->is_dense = false;
//      return tmp;
//    }
//    pcl::PointCloud<PointP>::Ptr generatePointCloudP(Mat& depth)
//    {
//      double fx = cameraMatrix.at<double>(0,0);
//      double fy = cameraMatrix.at<double>(1,1);
//      double cx = cameraMatrix.at<double>(0,2);
//      double cy = cameraMatrix.at<double>(1,2);
//      PointCloudP::Ptr tmp(new PointCloudP());
//      for(int m =0; m<depth.rows; m+=5)
//      {
//        for(int n = 0; n<depth.cols; n+=5)
//        {
//          float d = depth.ptr<float>(m)[n];
//          if(d<0.01 || d>6)
//            continue;
//          pcl::PointXYZ p;
//          p.z = d;
//          p.x = (n-cx)*d/fx;
//          p.y = (n-cy)*d/fy;

//          tmp->points.push_back(p);

//        }
//      }
//      tmp->is_dense = false;
//      return tmp;
//    }

//    pcl::PointCloud<PointP>::Ptr transformPointCloud(SE3 pose, pcl::PointCloud<PointP>& pointCloudP)
//    {
//      PointCloudP::Ptr cloud(new PointCloudP);
//      pcl::transformPointCloud(pointCloudP, *cloud, pose.matrix().inverse().matrix());// transform from cam to world
//      cloud->is_dense = false;
//      return cloud;
//    }
//    pcl::PointCloud<PointI>::Ptr transformPointCloudI(SE3 pose, pcl::PointCloud<PointI>& pointCloudI)
//    {
//      PointCloudI::Ptr cloud(new PointCloudI);
//      pcl::transformPointCloud(pointCloudI, *cloud, pose.matrix().inverse().matrix());// transform from cam to world
//      cloud->is_dense = false;
//      return cloud;
//    }
//    pcl::PointCloud<PointRGB>::Ptr transformPointCloudRGB(SE3 pose, pcl::PointCloud<PointRGB>& pointCloudRGB)
//    {
//      PointCloudRGB::Ptr cloud(new PointCloudRGB);
//      pcl::transformPointCloud(pointCloudRGB, *cloud, pose.matrix().inverse().matrix());// transform from cam to world
//      cloud->is_dense = false;
//      return cloud;
//    }

//    void viewer()
//    {
//      pcl::visualization::CloudViewer viewer("point cloud viewer");

//    }
//    octomap::Pointcloud generateOctomapPC(Mat& depth)
//    {
//      double fx = cameraMatrix.at<double>(0,0);
//      double fy = cameraMatrix.at<double>(1,1);
//      double cx = cameraMatrix.at<double>(0,2);
//      double cy = cameraMatrix.at<double>(1,2);
//      octomap::Pointcloud tmp;
//      for(int m =0; m<depth.rows; m+=5)
//      {
//        for(int n = 0; n<depth.cols; n+=5)
//        {
//          float d = depth.ptr<float>(m)[n];
//          if(d<0.01 || d>6)
//            continue;

//          double z = d;
//          double x = (n-cx)*d/fx;
//          double y = (n-cy)*d/fy;

//          octomap::point3d p(x,y,z);

//          tmp.push_back(p);

//        }
//      }

//    }

//    octomap::Pointcloud transformOctomapPC(SE3 pose, octomap::Pointcloud octoP)
//    {
//      Vector3d trans = pose.translation();
//      Quaterniond rots = pose.unit_quaternion();

//      octomath::Vector3 tran(trans(0),trans(1),trans(2));
//      octomath::Quaternion rot(rots.w(),rots.x(),rots.y(),rots.z());

//      octomap::pose6d poseOcto(tran, rot);

//      octomap::Pointcloud tmp(octoP);
//      tmp.transform(poseOcto.inv());

//      return tmp;

//    }

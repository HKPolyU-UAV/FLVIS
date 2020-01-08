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

//octomap::point3d origin(0.0,0.0,0.0);
//cout<<"generate octo pc"<<endl;
//tic_toc_ros octo_insertion;
//shared_ptr<octomap::Pointcloud> octomapTmpCloud = make_shared<octomap::Pointcloud>();//(new octomap::Pointcloud());
//octomapTmpCloud->reserve(tmpPtrW->size());
//tic_toc_ros get_octo_points;
//for (PointCloudP::const_iterator it = tmpPtrW->begin(); it != tmpPtrW->end(); ++it){
//  if (!std::isnan(it->z)) octomapTmpCloud->push_back(it->x, it->y, it->z);
//}
//get_octo_points.toc();
//Vector3d trans = kf_lc_ptr->T_c_w.translation();
//Quaterniond rots = kf_lc_ptr->T_c_w.unit_quaternion();
//float x = trans(0);
//float y = trans(1);
//float z = trans(2);
//float qw = rots.w();
//float qx = rots.x();
//float qy = rots.y();
//float qz = rots.z();
//octomath::Vector3 tran(x,y,z);
//octomath::Quaternion rot(qw,qx,qy,qz);


//octomap::pose6d poseOcto(tran, rot);
//octomap::point3d origin = poseOcto.inv().trans();

//tic_toc_ros octo_in;
//globalOctreePtr->insertPointCloud(*octomapTmpCloud, origin);
//octo_in.toc();
//tic_toc_ros octo_update;
//globalOctreePtr->updateInnerOccupancy();
//octo_update.toc();

//cout<<"point cloud transfrom and insertion";
//octo_insertion.toc();
//cout<<"write octo pc "<<endl;
//globalOctreePtr->writeBinary("/home/lsgi/out/globalOcree.bt");

//        if(kf_curr_idx > lcKFStart) bool is_lc = isLoopClosure(kf_map[kf_curr_idx-5],kf_map[kf_curr_idx],loop_pose);
//        {
//          tt_le.toc();
//          tic_toc_ros tt_pgo;
//          loop_ids.push_back(Vec3I(kf_curr_idx-5, kf_curr_idx, 1));
//          loop_poses.push_back(loop_pose);
//          loopClosureOnCovGraphG2O();
//          tt_pgo.toc();
//        }
//        if(kf_curr_idx == 200)
//        {

//          for(size_t i = 0; i<kf_map_lc.size();i++)
//          {
//            pcl::PointCloud<PointP>::Ptr tmpPC1 = generatePointCloudP(kf_map_lc[i]->d_img);
//            pcl::PointCloud<PointP>::Ptr tmpPCinMap = transformPointCloudP(kf_map_lc[i]->T_c_w, *tmpPC1);
//            pcl::PointCloud<PointP>::Ptr tmpPCinOdom = transformPointCloudP(kf_map_lc[i]->T_c_w_odom, *tmpPC1);
//            *globalafterPC += *tmpPCinMap;
//            *globalvoPC += *tmpPCinOdom;
//          }
//          pcl::io::savePLYFileBinary("/home/lsgi/out/aftermap.ply", *globalafterPC);
//          pcl::io::savePLYFileBinary("/home/lsgi/out/vo.ply", *globalvoPC);
//        }


//tic_toc_ros pc_insertion;
//pcl::PointCloud<PointP>::Ptr tmpPtr = generatePointCloudP(kf_lc_ptr->d_img);
//pcl::PointCloud<PointP>::Ptr tmpPtrW = transformPointCloudP(kf_lc_ptr->T_c_w, *tmpPtr);
//*globalPC += *tmpPtrW;
//pcl::io::savePLYFileBinary("/home/lsgi/out/map.ply", *globalPC);
//cout<<"point cloud transfrom and insertion: ";
//pc_insertion.toc();

//tic_toc_ros bow_save_tt;
//std::ofstream of;
//of.open("/home/lsgi/out/sim_mat.txt");

//for(size_t i = 0; i < sim_matrix.size(); i++){
//  for(size_t j = 0;j < sim_matrix.size(); j++){
//    of<<sim_matrix[i][j]<<" ";
//  }
//  of << "\n";
//}

//of.close();

//cout<<"bow save cost: ";bow_save_tt.toc();

// Publish tf transform
//      static tf2_ros::TransformBroadcaster br;
//      geometry_msgs::TransformStamped transformStamped;
//      transformStamped.header.stamp = tt;
//      transformStamped.header.frame_id = "kinect2_rgb_optical_frame";
//      transformStamped.child_frame_id = "map";
//      transformStamped.transform.translation.x = tcw.at<float>(0);
//      transformStamped.transform.translation.y = tcw.at<float>(1);
//      transformStamped.transform.translation.z = tcw.at<float>(2);
//      vector<float> q = ORB_SLAM2::Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
//      transformStamped.transform.rotation.x = q[0];
//      transformStamped.transform.rotation.y = q[1];
//      transformStamped.transform.rotation.z = q[2];
//      transformStamped.transform.rotation.w = q[3];

//      br.sendTransform(transformStamped);

//mapping
//PointCloudP::Ptr globalPC = boost::make_shared<PointCloudP>();
//PointCloudP::Ptr globalafterPC = boost::make_shared<PointCloudP>();
//PointCloudP::Ptr globalvoPC = boost::make_shared<PointCloudP>();
//shared_ptr<octomap::OcTree> globalOctreePtr = make_shared<octomap::OcTree>(0.4);
////octomap::OcTree globalOctree = octomap::OcTree(0.05);
//pcl::PointCloud<PointP>::Ptr generatePointCloudP(Mat& depth)
//{
//  PointCloudP::Ptr tmp(new PointCloudP());
//  for(int m =0; m<depth.rows; m+=5)
//  {
//    for(int n = 0; n<depth.cols; n+=5)
//    {
//      float d = static_cast<float>(depth.ptr<ushort>(m)[n]/1000.0);
//      if(d<0.01f || d>4.0f)
//        continue;
//      pcl::PointXYZ p;
//      p.z = d;
//      p.x = (n-static_cast<float>(cx))*d/static_cast<float>(fx);
//      p.y = (m-static_cast<float>(cy))*d/static_cast<float>(fy);

//      tmp->points.push_back(p);

//    }
//  }
//  tmp->is_dense = false;
//  return tmp;
//}


//pcl::PointCloud<PointP>::Ptr transformPointCloudP(SE3 pose, pcl::PointCloud<PointP>& pointCloudP)
//{
//  PointCloudP::Ptr cloud(new PointCloudP);
//  Matrix<double,4,4> transformMat = pose.matrix().inverse();
//  Matrix<float,4,4> transformMatD = transformMat.cast<float>();
//  pcl::transformPointCloud(pointCloudP, *cloud, transformMatD);// transform from cam to world
//  cloud->is_dense = false;
//  return cloud;
//}

// shared_ptr<KeyFrameStruct> kf_ptr = make_shared<KeyFrameStruct>(kf);
 // new feature detector and descriptor
 //tic_toc_ros feature_tt;
 //Ptr<FastFeatureDetector> detector= FastFeatureDetector::create();
 //Ptr<FeatureDetector> detector = ORB::create(1000);
 //vector<KeyPoint> FASTFeatures;
 //vector<Point2f>  kps;
// detector->detect(kf.img, FASTFeatures);
 //KeyPoint::convert(FASTFeatures,kps);
// Mat tmpDescriptors;
 //vector<Mat> newDescriptors;
 //newDescriptors.clear();

// Ptr<DescriptorExtractor> extractor = ORB::create(1000);
 //extractor->compute(kf.img, FASTFeatures, tmpDescriptors);
 //descriptors_to_vMat(tmpDescriptors,newDescriptors);

// cout<<"descriptor numbers: "<<newDescriptors.size()<<endl;
// cout<<"feature cost: ";feature_tt.toc();tic_toc_ros bow_tt;

 //size_t DesSize = 800;
// if (newDescriptors.size() < DesSize) DesSize = newDescriptors.size();
// vector<Mat> DesTmp(newDescriptors.begin(),newDescriptors.begin()+static_cast<long>(DesSize)-1);

//save to new kf vector
//shared_ptr<KeyFrameLC> kf_lc_ptr =std::make_shared<KeyFrameLC>();
//kf_lc_ptr->t = ros::Time::now();

//kf_lc_ptr->frame_id = kf_ptr->frame_id;
//kf_lc_ptr->d_img = kf_ptr->d_img;
//kf_lc_ptr->keyframe_id = kf_id;
//vector<Vec2> lm_2d;
//vector<double> lm_d;
//for(size_t i = 0; i<FASTFeatures.size();i++)
//{
//  Point2f cvtmp = FASTFeatures[i].pt;
//  Vec2 tmp(cvtmp.x,cvtmp.y);
//  double d = (kf_lc_ptr->d_img.at<ushort>(cvtmp))/1000;
//  lm_2d.push_back(tmp);
//  lm_d.push_back(d);
//}
//kf_lc_ptr->lm_2d = lm_2d;
//kf_lc_ptr->lm_d = lm_d;
//kf_lc_ptr->lm_count = static_cast<int>(lm_2d.size());
//cout<<"pass feature number: "<<kf.lm_count;
//lm_2d.clear();
//lm_d.clear();
//kf_lc_ptr->lm_descriptor = newDescriptors;
//kf_lc_ptr->kf_bv = kf_bv;
//kf_lc_ptr->T_c_w_odom = kf_ptr->T_c_w;
//kf_lc_ptr->T_c_w = kf_ptr->T_c_w*T_odom_map;// T_c_w c ccurrent camera, w odom,
//kf_id +=1;
//FASTFeatures.clear();
//kps.clear();
//newDescriptors.clear();

//voc.transform(kf_ptr->lm_descriptor,kf_bv);
//kf_map.push_back(kf_ptr);



//Ptr<FeatureDetector> detector = ORB::create(200);
//vector<KeyPoint> ORBFeatures;
//vector<Point2f>  kps;
//Mat ORBDescriptorsL;
//vector<Mat> ORBDescriptors;



//kps.clear();
//ORBFeatures.clear();
//ORBDescriptors.clear();


//detector->detect(img_unpack, ORBFeatures);
//KeyPoint::convert(ORBFeatures,kps);

//Ptr<DescriptorExtractor> extractor = ORB::create(200);
//extractor->compute(img_unpack, ORBFeatures, ORBDescriptorsL);
//descriptors_to_vMat(ORBDescriptorsL,ORBDescriptors);
//kf.lm_descriptor = ORBDescriptors;

//cout<<"descriptor numbers: "<<ORBDescriptors.size()<<endl;
//cout<<"feature cost: ";feature_tt.toc();

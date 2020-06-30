#include <manipulation_vision/search.hpp>

namespace manipulation
{

//----------------------------------------------------------------------------
/**
 * Constructor
 */
Search::Search()
{
    //server (flower search using end effector camera)
    searchServ = nh.advertiseService("search", &Search::search, this);
    searchFFServ = nh.advertiseService("searchFF", &Search::searchFF, this);

    //clients segmentation
    segmentFlowersClient =
    nh.serviceClient<manipulation_vision::SegmentFlowers>("/segment_image");
    segmentFlowersFFClient =
    nh.serviceClient<manipulation_vision::SegmentFlowersFF>("/segment_image_ff");

    //clients classification
    classifyFlowersClient =
    nh.serviceClient<manipulation_vision::ClassifyFlowers>("/classify_image");
    classifyPoseClient =
    nh.serviceClient<manipulation_vision::ClassifyPose>("/classify_pose");

    //clients mapping
    addFlowersToMapClient =
    nh.serviceClient<manipulation_common::UpdateFlowerMap>("/flower_mapper/update_flower_mapper");
    
    //debugging
    pubSeg = nh.advertise<sensor_msgs::PointCloud2>("/debug/segments", 0, true);
}

//----------------------------------------------------------------------------
/**
 * Server for performing search on image and depth image where the pose is
 * added to map for detected flowers (by passing the result to the factor
 * graph node, which is used for maintaining the map). We filter the rgb
 * image by removing points based on depth (e.g. we can't pollinate anything
 * that is out of the arms configuration space)
 *
 * Note we need to sync the images and camera info if we plan on
 * calling the search while the end-effector is in motion (for now we are
 * assuming the end-effector is stationary during the search); however
 * if we run search in real time (during motion, we can use message_filters
 * and boost::bind to synchronizer callbacks. See depthcallback_test.cpp for
 * and example on how to do this
 */
bool Search::search(manipulation_common::SearchForFlowers::Request  &req,
                    manipulation_common::SearchForFlowers::Response &res)
{
  //republish data
  ROS_INFO("Republish image/depth data...");
  republish();

  //get package directory
  ROS_INFO("Searching...");
  std::string filepath = ros::package::getPath("manipulation_vision") + "/data";

  //load rgb image from topic
  sensor_msgs::Image::ConstPtr msg_rgb_ptr =
  ros::topic::waitForMessage<sensor_msgs::Image>
  ("/camera/color/image_raw", ros::Duration(1));

  if(msg_rgb_ptr == NULL)
  {
    ROS_ERROR("search: waitForMessage expired, rgb_image NULL");
    return false;
  }

  cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(msg_rgb_ptr, "bgr8");
  cv::imwrite(filepath + "/rgb.jpg", cv_rgb_ptr->image);

  // save all images during experiments (comment to disable)
  // const char *home_dir = getpwuid(getuid())->pw_dir;
  // std::string filename = std::string(home_dir) + "/manipulation_ws/search_images";
  // cv::imwrite(filename + "/image" + std::to_string(search_image_count_) + ".jpg", cv_rgb_ptr->image);
  // ++search_image_count_;

  //load depth image from topic
  sensor_msgs::Image::ConstPtr msg_depth_ptr =
  ros::topic::waitForMessage<sensor_msgs::Image>
  ("/camera/aligned_depth_to_color/image_raw", ros::Duration(1));

  if(msg_depth_ptr == NULL)
  {
    ROS_ERROR("search: waitForMessage expired, depth_image NULL");
    return false;
  }

  //convert depth data type
  //TODO: need to check the encoding type, often it is 16UC1
  cv_bridge::CvImagePtr cv_depth_ptr =
  cv_bridge::toCvCopy(msg_depth_ptr, "16UC1");

  //load depth info
  //TODO: i broke this on purpose -nwh
  sensor_msgs::CameraInfo::ConstPtr msg_depth_info_ptr =
  ros::topic::waitForMessage<sensor_msgs::CameraInfo>
  ("/camera/color/camera_info", ros::Duration(1));

  if(msg_depth_info_ptr == NULL)
  {
    ROS_ERROR("search: waitForMessage expired, msg_depth_info_ptr NULL");
    return false;
  }

  //for some u, v coordinate in our image, we can get the x, y, z using
  //the intrinsic parameters (for the realsense the depth image is in
  //millimeters, so we divide by 1000 to get meters)
  // float fx = msg_depth_info_ptr->K[0];
  // float fy = msg_depth_info_ptr->K[4];
  // float cx = msg_depth_info_ptr->K[2];
  // float cy = msg_depth_info_ptr->K[5];
  // float depth = image_depth.at<short int>(cv::Point(u,v)) / 1000.0;
  // float x = (x - cx) * depth / fx;
  // float y = (y - cy) * depth / fy;
  // float z = depth;

  //depth constraint (ignore points far aware)
  depth_constraint(cv_rgb_ptr->image, cv_depth_ptr->image);

  //save rgb image (for segmentation)
  //TODO: consider using nodelets or combing class to reduce writing
  //      to disk or passing data over TCP/IP (would require
  //      a lot of modifications to code)
  cv::imwrite(filepath + "/rgb_constrained.jpg", cv_rgb_ptr->image);
  cv::imwrite(filepath + "/depth.jpg",  cv_depth_ptr->image);

  //display images
  // cv::namedWindow("rgb (depth constrained)", cv::WINDOW_NORMAL);
  // cv::imshow("rgb (depth constrained)", cv_rgb_ptr->image);
  // cv::waitKey(0);

  //do segmentation
  segmentFlowersSrv.request.filepath = filepath + "/rgb.jpg";
  if(segmentFlowersClient.call(segmentFlowersSrv))
  {
    if(segmentFlowersSrv.response.success == true)
    {
      ROS_INFO("SegmentFlowers call successful!");
    }
    else
    {
      ROS_ERROR("segmentFlowersSrv.success == false");
    }
  }
  else
  {
    ROS_ERROR("Error! Failed to call service SegmentFlowers");
  }

  //do classification
  classifyFlowersSrv.request.numberOfSegments =
  segmentFlowersSrv.response.numberOfSegments;
  if(classifyFlowersClient.call(classifyFlowersSrv))
  {
   if(classifyFlowersSrv.response.success == true)
    {
      ROS_INFO("ClassifyFlowers call successful!");
    }
    else
    {
      ROS_ERROR("classifyFlowersSrv.success == false");
    }
  }
  else
  {
    ROS_ERROR("Error! Failed to call service ClassifyFlowers");
  }

  //compute position of each flower
  std::vector<geometry_msgs::PoseStamped> obs; obs.clear();
  for(int i=0; i<classifyFlowersSrv.response.responseProbabilities.size(); i++)
  {
    if(classifyFlowersSrv.response.responseProbabilities[i] > 0.7)
    {
      //check if size of flower is reasonable
      //TODO: use meters instead of using pixels
      int w = segmentFlowersSrv.response.width[i];
      int h = segmentFlowersSrv.response.height[i];
      if(w > 150 || h > 150 || w < 15 || h < 15)
      {
        std::cout << "removed " << i << " due to size" << std::endl;
        continue;
      }

      //image coordinates of flower
      double u = segmentFlowersSrv.response.u[i];
      double v = segmentFlowersSrv.response.v[i];

      //xyz
      geometry_msgs::Point point;
      point.z = cv_depth_ptr->image.at<short int>( cv::Point(u,v) ) / 1000.0;
      point.x = (u - msg_depth_info_ptr->K[2]) * point.z / msg_depth_info_ptr->K[0];
      point.y = (v - msg_depth_info_ptr->K[5]) * point.z / msg_depth_info_ptr->K[4];
      std::cout << "search: (i, x, y, z, p) = " << i << ", "
                                                << point.x << ", "
                                                << point.y << ", "
                                                << point.z << ", "
                                                << classifyFlowersSrv.response.responseProbabilities[i]
                                                << std::endl;

      //quat
      geometry_msgs::Quaternion quat;

      //define pose stamped from quat and point
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = msg_depth_info_ptr->header.stamp;
      pose.header.frame_id = msg_depth_info_ptr->header.frame_id;
      pose.pose.position = point;
      pose.pose.orientation = quat;

      //transform pose
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tf2_listener(tfBuffer);
      geometry_msgs::TransformStamped T_w2c; //transform from camera to world
      T_w2c = tfBuffer.lookupTransform("world", "camera_link", ros::Time(0), ros::Duration(1.0)); //
      // std::cout << "T_w2c = " << T_w2c << std::endl;

      geometry_msgs::PoseStamped pose_new;
      tf2::doTransform(pose, pose_new, T_w2c);

      std::cout << "world: (x, y, z) = " << pose_new.pose.position.x << ", "
                                         << pose_new.pose.position.y << ", "
                                         << pose_new.pose.position.z << ", "
                                         << std::endl;

    //(placeholder for global orientation)
     pose_new.pose.orientation.x = -0.7071068;
     pose_new.pose.orientation.y = 0.0;
     pose_new.pose.orientation.z = 0.0;
     pose_new.pose.orientation.w = 0.7071068;

     double change_deg = 15; // degrees
     double crad = change_deg * M_PI/180;

     tf2::Quaternion orig_quat(pose_new.pose.orientation.x, pose_new.pose.orientation.y, pose_new.pose.orientation.z, pose_new.pose.orientation.w);

     tf2::Quaternion rot_quat;
     rot_quat.setRPY(crad, 0, 0);
     tf2::Quaternion new_quat = rot_quat * orig_quat;

     tf2::convert(new_quat, pose_new.pose.orientation);

     // //convert to eigen (for transform)
     // Eigen::Quaterniond q(T_w2c.transform.rotation.w,
     //                      T_w2c.transform.rotation.x,
     //                      T_w2c.transform.rotation.y,
     //                      T_w2c.transform.rotation.z);
     // // q.x = T_w2c.transform.rotation.x;
     // // q.y = T_w2c.transform.rotation.y;
     // // q.z = T_w2c.transform.rotation.z;
     // // q.w = T_w2c.transform.rotation.w;
     // Eigen::Matrix3d eigen_R = q.normalized().toRotationMatrix();
     // std::cout << "eigen_R = " << eigen_R << std::endl;
     //
     // Eigen::Vector3d eigen_t;
     // eigen_t << T_w2c.transform.translation.x, T_w2c.transform.translation.y, T_w2c.transform.translation.z;
     //
     // Eigen::Vector3d eigen_pose;
     // eigen_pose << point.x, point.y, point.z;
     //
     // //do transform
     // Eigen::Vector3d eigen_pose_new = eigen_R*eigen_pose + eigen_t;
     //
     // std::cout << "eigen: (x, y, z) = " << eigen_pose_new(0) << ", "
     //                                    << eigen_pose_new(1) << ", "
     //                                    << eigen_pose_new(2) << ", "
     //                                    << std::endl;

      //add stamped pose to observations
      // also check if depth is reasonable, must check again (in additiona to
      // depth constraint since flower may only be partially segmented resulting
      // in poor estimate of pose)
      if(point.z < 0.75 && point.z > 0.05)
      {
        obs.push_back(pose_new);
      }
    }
  }



  //compute pose of each flower (fill poses with estimated quat)
  // TODO: This step.

  //update flower map (using mapper service)
  updateFlowerMapSrv.request.poses = obs;
  if(addFlowersToMapClient.call(updateFlowerMapSrv))
  {
    if(updateFlowerMapSrv.response.status == 1)
    {
      ROS_INFO("UpdateFlowerMap call successful!");
    }
    else
    {
      ROS_ERROR("updateFlowerMapSrv.response.status == 0");
    }
  }
  else
  {
    ROS_ERROR("Error! Failed to call service UpdateFlowerMap");
  }

  //update response
  res.success = true;

  return true;
}

//----------------------------------------------------------------------------
/**
 * Server for performing search on image and depth image where the pose is
 * added to map for detected flowers (by passing the result to the factor
 * graph node, which is used for maintaining the map). We filter the rgb
 * image by removing points based on depth (e.g. we can't pollinate anything
 * that is out of the arms configuration space)
 *
 * Note we need to sync the images and camera info if we plan on
 * calling the search while the end-effector is in motion (for now we are
 * assuming the end-effector is stationary during the search); however
 * if we run search in real time (during motion, we can use message_filters
 * and boost::bind to synchronizer callbacks. See depthcallback_test.cpp for
 * and example on how to do this
 *
 * Note: The difference between searchFF and search is that searchFF uses the
 * segmentFlowerFF, which segments all points contained in the flower instead 
 * of segmenting only the bounding box containing the flower.
 *
 * TODO: consider using nodelets or combing class to reduce writing to disk 
 *       or passing data over TCP/IP (however, this would require a lot of 
 *       modifications to code)
 */
bool Search::searchFF(manipulation_common::SearchForFlowers::Request  &req,
                      manipulation_common::SearchForFlowers::Response &res)
{
  //republish data
  ROS_INFO("Republish image/depth data...");
  if(!republish())
  {
    ROS_ERROR("Republish failed! Are all camera topics publishing?");
    return false;
  }

  //get package directory
  ROS_INFO("Searching (using FF)...");
  std::string filepath = ros::package::getPath("manipulation_vision") + "/data";

  //load rgb and depth
  _load_rgb("/camera/color");
  _load_depth("/camera/aligned_depth_to_color");

  //depth constraint (ignore points far aware)
  depth_constraint(_rgb, _depth);

  //do segmentation
  _do_segmentation();
  _do_classification();

  //compute point cloud for each segment
  std::vector<sensor_msgs::PointCloud2> point_clouds;
  std::vector<geometry_msgs::PoseStamped> poses;
  for(int i=0; i<classifyFlowersSrv.response.responseProbabilities.size(); i++)
  {
    //load segment
    manipulation_vision::Segment segment 
    = segmentFlowersFFSrv.response.segments[i];

    //check if flower (i.e., probability above threshold)
    if(classifyFlowersSrv.response.responseProbabilities[i] < 0.7)
      continue;

    //check if size of flower is reasonable
    //if(!_size_constraint_satisfied(segment.width, segment.height))
    //  continue;

    //display probability of ith detected flower
    ROS_INFO("searchFF: (i, prob) = %i, %f", i, classifyFlowersSrv.response.responseProbabilities[i]);
    
    //compute point cloud for each segment (from depth image)
    sensor_msgs::PointCloud2 point_cloud = _compute_point_cloud(segment);
    
    //compute pose of of flower (for now, only position using centroid of xyz)
    geometry_msgs::PoseStamped pose = _compute_pose(point_cloud);

    //ignore if too far away (in base link reference frame)
    if(!_distance_constraint_satisfied(pose))
      continue;

    //add pose and points to list
    point_clouds.push_back(point_cloud);
    poses.push_back(pose);
  }

  //publish point cloud of all segments for visualization
  _publish_segments(point_clouds);

  //call server for updating flower map
  updateFlowerMapSrv.request.poses = poses;
  updateFlowerMapSrv.request.point_clouds = point_clouds;
  if(addFlowersToMapClient.call(updateFlowerMapSrv))
  {
    if(updateFlowerMapSrv.response.status == 1)
    {
      ROS_INFO("UpdateFlowerMap call successful!");
    }
    else
    {
      ROS_ERROR("updateFlowerMapSrv.response.status == 0");
    }
  }
  else
  {
    ROS_ERROR("Error! Failed to call service UpdateFlowerMap");
  }

  return true;
}


//----------------------------------------------------------------------------
/**
 * 
 */
bool Search::_load_rgb(std::string topic)
{
  //image topic
  std::string topic_image = topic + "/image_raw";

  //wait for message
  sensor_msgs::Image::ConstPtr msg_rgb_ptr =
  ros::topic::waitForMessage<sensor_msgs::Image>(topic_image, ros::Duration(1));

  if(msg_rgb_ptr == NULL)
  {
    ROS_ERROR("search: waitForMessage expired, rgb_image NULL");
    return false;
  }

  //convert to opencv type (for rgb image)
  cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(msg_rgb_ptr, "bgr8");
  _rgb = cv_rgb_ptr->image.clone();

  //write rgb to file
  std::string filepath = ros::package::getPath("manipulation_vision") + "/data";
  cv::imwrite(filepath + "/rgb.jpg", _rgb);

  //info topic
  std::string topic_info = topic + "/camera_info";

  //load info
  sensor_msgs::CameraInfo::ConstPtr msg_rgb_info_ptr =
  ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic_info, ros::Duration(1));

  if(msg_rgb_info_ptr == NULL)
  {
    ROS_ERROR("search: waitForMessage expired, msg_rgb_info_ptr NULL");
    return false;
  }

  _rgb_info = *msg_rgb_info_ptr;
}

//----------------------------------------------------------------------------
/**
 * 
 */
bool Search::_load_depth(std::string topic)
{
  //image topic
  std::string topic_image = topic + "/image_raw";

  //wait for message
  sensor_msgs::Image::ConstPtr msg_depth_ptr =
  ros::topic::waitForMessage<sensor_msgs::Image>(topic_image, ros::Duration(1));

  if(msg_depth_ptr == NULL)
  {
    ROS_ERROR("search: waitForMessage expired, depth_image NULL");
    return false;
  }

  //convert to opencv type (for depth image)
  //TODO: check the encoding type, often 16UC1 (so using this for now)
  cv_bridge::CvImagePtr cv_depth_ptr =
  cv_bridge::toCvCopy(msg_depth_ptr, "16UC1");
  _depth = cv_depth_ptr->image.clone();

  //write depth to file (although the encoding is messed up for .jpg)
  std::string filepath = ros::package::getPath("manipulation_vision") + "/data";
  cv::imwrite(filepath + "/depth.jpg",  _depth);

  //info topic
  //TODO: i broke this on purpose -nwh
  //std::string topic_info = topic + "/camera_info";
  std::string topic_info = "/camera/color/camera_info";

  //load info
  sensor_msgs::CameraInfo::ConstPtr msg_depth_info_ptr =
  ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic_info, ros::Duration(1));

  if(msg_depth_info_ptr == NULL)
  {
    ROS_ERROR("search: waitForMessage expired, msg_depth_info_ptr NULL");
    return false;
  }

  _depth_info = *msg_depth_info_ptr;
}

//----------------------------------------------------------------------------
/**
 * 
 */
sensor_msgs::PointCloud2 Search::_compute_point_cloud (manipulation_vision::Segment segment)
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_pc;
  for(int j = 0; j < segment.numberOfPoints; j++) //for all points in segment
  {
    double u = segment.u[j];
    double v = segment.v[j];

    //xyzrgb
    pcl::PointXYZRGB point;
    point.z = _depth.at<short int>( cv::Point(u,v) ) / 1000.0;
    point.x = (u - _depth_info.K[2]) * point.z / _depth_info.K[0];
    point.y = (v - _depth_info.K[5]) * point.z / _depth_info.K[4];
    point.r = segment.r[j];
    point.g = segment.g[j];
    point.b = segment.b[j];
    
    //add point to point cloud
    pcl_pc.points.push_back(point);
  }

  //convert pcl::PointCloud<pcl::PointXYZRGB> to sensor_msgs::PointCloud2
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_pc,pcl_pc2);
  sensor_msgs::PointCloud2 ros_pc2;
  pcl_conversions::moveFromPCL(pcl_pc2, ros_pc2);
  ros_pc2.header = _depth_info.header;

  return ros_pc2;
}

//----------------------------------------------------------------------------
/**
 * 
 */
geometry_msgs::PoseStamped Search::_compute_pose(sensor_msgs::PointCloud2 ros_pc2)
{

  //convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::moveToPCL(ros_pc2, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB> pcl_pc;
  pcl::fromPCLPointCloud2(pcl_pc2, pcl_pc);

  //compute centroid
  double mu_x = 0;
  double mu_y = 0;
  double mu_z = 0;
  for(int i = 0; i < pcl_pc.size(); i++)
  {
    mu_x += pcl_pc.at(i).x;
    mu_y += pcl_pc.at(i).y;
    mu_z += pcl_pc.at(i).z;
  }

  //set position as centroid
  geometry_msgs::PoseStamped pose;
  pose.header = ros_pc2.header;
  pose.pose.position.x = mu_x/(double)pcl_pc.size();
  pose.pose.position.y = mu_y/(double)pcl_pc.size();
  pose.pose.position.z = mu_z/(double)pcl_pc.size();

  return pose;
}

//----------------------------------------------------------------------------
/**
 * 
 */
bool Search::_do_segmentation()
{
  std::string filepath = ros::package::getPath("manipulation_vision") + "/data";
  segmentFlowersFFSrv.request.filepath = filepath + "/rgb.jpg";
  if(segmentFlowersFFClient.call(segmentFlowersFFSrv))
  {
    if(segmentFlowersFFSrv.response.success == true)
    {
      ROS_INFO("SegmentFlowersFF call successful!");
    }
    else
    {
      ROS_ERROR("segmentFlowersFFSrv.success == false");
    }
  }
  else
  {
    ROS_ERROR("Error! Failed to call service SegmentFlowersFF");
  }
}

//----------------------------------------------------------------------------
/**
 * 
 */
bool Search::_do_classification()
{
  classifyFlowersSrv.request.numberOfSegments =
  segmentFlowersFFSrv.response.numberOfSegments;
  if(classifyFlowersClient.call(classifyFlowersSrv))
  {
   if(classifyFlowersSrv.response.success == true)
    {
      ROS_INFO("ClassifyFlowers call successful!");
    }
    else
    {
      ROS_ERROR("classifyFlowersSrv.success == false");
    }
  }
  else
  {
    ROS_ERROR("Error! Failed to call service ClassifyFlowers");
  }
}




//----------------------------------------------------------------------------
/**
 * Removes pixels from the rgb image (i.e. assigns values of 0 to r, g, and b)
 * based on the depth
 */
bool Search::depth_constraint( cv::Mat & rgb,
                               cv::Mat & depth)
{
  //criteria
  double criteria = 0.75;

  //filter rgb
  //TODO: change to pointers for efficiency
  for(int v=0; v < rgb.rows; v++)
  {
    for(int u=0; u < rgb.cols; u++)
    {
      //compute depth
      float z = depth.at<short int>( cv::Point(u,v) ) / 1000.0;
      if(false) //depth constraint
      {
        rgb.at<cv::Vec3b>(cv::Point(u,v)) = cv::Vec3b(50,100,50);
      }
    }
  }

  //write image to file
  std::string filepath = ros::package::getPath("manipulation_vision") + "/data";
  cv::imwrite(filepath + "/rgb_constrained.jpg", rgb);

  return true;
}

//----------------------------------------------------------------------------
/**
 * 
 */
bool Search::_size_constraint_satisfied ( int width,
                                          int height)
{
  if(width > 150 || height > 150 || width < 15 || height < 15) //manually parameterized
  {
    return false;
  }
  return true;
}

//----------------------------------------------------------------------------
/**
 * 
 */
bool Search::_distance_constraint_satisfied (geometry_msgs::PoseStamped pose)
{
  // geometry_msgs::PoseStamped pose_transformed;
  // listener("world", pose, pose_transformed);
  return true;
}

//----------------------------------------------------------------------------
/**
 * 
 */
bool Search::republish()
{
  ros::Publisher pub_depth_info = nh.advertise<sensor_msgs::CameraInfo>("/data/camera/aligned_depth_to_color/camera_info", 1);
  ros::Publisher pub_color_info = nh.advertise<sensor_msgs::CameraInfo>("/data/camera/color/camera_info", 1);
  ros::Publisher pub_depth_img = nh.advertise<sensor_msgs::Image>("/data/camera/aligned_depth_to_color/image_raw", 1);
  ros::Publisher pub_color_img = nh.advertise<sensor_msgs::Image>("/data/camera/color/image_raw", 1);
  ros::Publisher pub_cloud = nh.advertise<pcl::PCLPointCloud2>("/data/camera/depth/color/points", 1);
  ros::Publisher pub_tf = nh.advertise<tf2_msgs::TFMessage>("/data/tf", 1);
  ros::Publisher pub_tf_static = nh.advertise<tf2_msgs::TFMessage>("/data/tf_static", 1);

  sensor_msgs::Image::ConstPtr msg_depth_ptr = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", ros::Duration(1));
  sensor_msgs::Image::ConstPtr msg_color_ptr = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw", ros::Duration(1));
  //TODO: i broke this on purpose -nwh
  //sensor_msgs::CameraInfo::ConstPtr msg_depth_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/aligned_depth_to_color/camera_info", ros::Duration(1));
  sensor_msgs::CameraInfo::ConstPtr msg_color_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info", ros::Duration(1));
  sensor_msgs::CameraInfo::ConstPtr msg_depth_info_ptr = msg_color_info_ptr;
  pcl::PCLPointCloud2::ConstPtr msg_cloud_ptr = ros::topic::waitForMessage<pcl::PCLPointCloud2> ("/camera/depth/color/points", ros::Duration(1));
  tf2_msgs::TFMessage::ConstPtr msg_tf = ros::topic::waitForMessage<tf2_msgs::TFMessage> ("/tf", ros::Duration(1));
  tf2_msgs::TFMessage::ConstPtr msg_tf_static = ros::topic::waitForMessage<tf2_msgs::TFMessage> ("/tf_static", ros::Duration(1));

  // Check for null ptrs so node doesn't die if one topic is missing
  if(msg_depth_ptr == NULL ||
     msg_color_ptr == NULL ||
     msg_depth_info_ptr == NULL ||
     msg_color_info_ptr == NULL)
  {
    return false;
  }


  pub_depth_img.publish(msg_depth_ptr);
  pub_color_img.publish(msg_color_ptr);
  pub_depth_info.publish(msg_depth_info_ptr);
  pub_color_info.publish(msg_color_info_ptr);
  pub_cloud.publish(msg_cloud_ptr);
  pub_tf.publish(msg_tf);
  pub_tf_static.publish(msg_tf_static);

  return true;
}

//----------------------------------------------------------------------------
/**
 * 
 */
bool Search::_publish_segments(std::vector<sensor_msgs::PointCloud2> points_clouds)
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_merged_pc;
  for(int i = 0; i < points_clouds.size(); i++) //for all points in segment
  {
    //convert ith sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB> 
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::moveToPCL(points_clouds[i], pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pcl::fromPCLPointCloud2(pcl_pc2,pc);
    
    //loop over points
    for(int j = 0; j < pc.size(); j++)
    {
      pcl_merged_pc.push_back(pc.at(j));
    }
  }

  //convert pcl::PointCloud<pcl::PointXYZRGB> to sensor_msgs::PointCloud2
  pcl::PCLPointCloud2 pcl_merged_pc2;
  pcl::toPCLPointCloud2(pcl_merged_pc, pcl_merged_pc2);
  sensor_msgs::PointCloud2 ros_merged_pc2;
  pcl_conversions::moveFromPCL(pcl_merged_pc2, ros_merged_pc2);
  ros_merged_pc2.header = _depth_info.header;

  //publish merged point clouds
  pubSeg.publish(ros_merged_pc2);
}

}

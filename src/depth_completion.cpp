#include "sensor_processing/depth_completion.h"

#include <Eigen/Dense>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>

namespace sensor_processing
{
DepthCompletion::DepthCompletion(ros::NodeHandle nh) : 
  nh_(nh),
  exact_sync_(ExactPolicy(10),
  sub_pointcloud_, sub_left_color_camera_info_, sub_left_color_image_)
{
  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server values can be overwritten.
  dynamic_reconfigure::Server<sensor_processing::DepthCompletionParamsConfig>::CallbackType cb;
  cb = boost::bind(&DepthCompletion::configCallback, this, _1, _2);
  dr_srv_.setCallback(cb);

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  int diamond_kernel_size;
  int full_kernel_size;
  pnh.param("diamondKernelSize", diamond_kernel_size, diamond_kernel_size);
  pnh.param("fullKernelSize", full_kernel_size, full_kernel_size);
  diamond_kernel_size_ = diamond_kernel_size;
  full_kernel_size_ = convertFullKernelSize(full_kernel_size);

  sub_pointcloud_.subscribe(nh_, "/kitti/velo/pointcloud", 1);
  sub_left_color_camera_info_.subscribe(nh_, "/kitti/camera_color_left/camera_info", 1);
  sub_left_color_image_.subscribe(nh_, "/kitti/camera_color_left/image_raw", 1);
  exact_sync_.registerCallback(boost::bind(&DepthCompletion::callback,
                                              this, _1, _2, _3));

  pub_depth_image_ = nh_.advertise<Image>("/kitti/depth_image", 1);
  pub_completion_image_ = nh_.advertise<Image>("/kitti/completed_image", 1);
  pub_completed_pointcloud_ = nh_.advertise<PointCloud2>("/kitti/completed_pointcloud", 1);

}

bool DepthCompletion::inImage(const CameraInfoConstPtr& cam_info, const int u, const int v)
{
  return(u >= 0 && u < cam_info->height && v >= 0 && v < cam_info->width);
}

void DepthCompletion::depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols)
  {
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);
  }
  //The following doesn't work if there are NaNs
  double minVal, maxVal; 
  minMaxLoc(float_img, &minVal, &maxVal);
  ROS_INFO("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
  cv::convertScaleAbs(float_img, mono8_img, 2.55, 0.0);
}

void DepthCompletion::callback(const PointCloud2ConstPtr& pc_msg,
                               const CameraInfoConstPtr& l_info_msg,
                               const ImageConstPtr& l_image_msg)
{

  cv::Mat depth_image;
  pointCloudToDepthImage(pc_msg, l_info_msg, depth_image);
  cv::Mat depth_image_8;
  depthToCV8UC1(depth_image, depth_image_8);
  cv_bridge::CvImage cv_bridge_depth_image;
  cv_bridge_depth_image.image = depth_image_8;
  cv_bridge_depth_image.encoding = "mono8";
  cv_bridge_depth_image.header.stamp = l_info_msg->header.stamp;
  pub_depth_image_.publish(cv_bridge_depth_image.toImageMsg());


  cv::Mat depth_completion_image;
  processDepthCompletion(l_info_msg, depth_image, depth_completion_image);
  cv::Mat depth_completion_image_8;
  depthToCV8UC1(depth_completion_image, depth_completion_image_8);
  cv_bridge::CvImage cv_bridge_depth_completion_image;
  cv_bridge_depth_completion_image.image = depth_completion_image_8;
  cv_bridge_depth_completion_image.encoding = "mono8";
  cv_bridge_depth_completion_image.header.stamp = l_info_msg->header.stamp;
  pub_completion_image_.publish(cv_bridge_depth_completion_image.toImageMsg());


  PointCloud2 pc;
  pc.header.frame_id = pc_msg->header.frame_id;
  depthImageToPointCloud(depth_completion_image, l_info_msg, pc);
  pc.header.stamp = ros::Time::now();
  pub_completed_pointcloud_.publish(pc);
  
}

void DepthCompletion::pointCloudToDepthImage(
    const PointCloud2ConstPtr& pc,
    const CameraInfoConstPtr& cam_info,
    cv::Mat& depth_image)
{
  // 1. To cam frame
  PointCloud2 pc_cam;
  std::string target_frame = cam_info->header.frame_id;
  bool transformed = pcl_ros::transformPointCloud(target_frame, *pc, pc_cam, listener_);
  
  // 2. To rect cam frame
  PointCloud2 pc_rect_cam;
  Eigen::Matrix4f R = Eigen::Matrix4f();
  R << cam_info->R[0], cam_info->R[1], cam_info->R[2], 0,
       cam_info->R[3], cam_info->R[4], cam_info->R[5], 0,
       cam_info->R[6], cam_info->R[7], cam_info->R[8], 0,
                    0,              0,              0, 1;
  pcl_ros::transformPointCloud(R, pc_cam, pc_rect_cam);

  // 3. To image frame
  PointCloud2 pc_img;
  Eigen::Matrix4f P = Eigen::Matrix4f();
  P << cam_info->P[0], cam_info->P[1], cam_info->P[2], cam_info->P[3],
       cam_info->P[4], cam_info->P[5], cam_info->P[6], cam_info->P[7], 
       cam_info->P[8], cam_info->P[9], cam_info->P[10], cam_info->P[11],
                    0,              0,               0,               1;
  pcl_ros::transformPointCloud(P, pc_rect_cam, pc_img);

  // Init depth image
  if(depth_image.rows == 0 || depth_image.cols == 0)
  {
    depth_image = cv::Mat::zeros(cam_info->width, cam_info->height, CV_32FC1);
  }

  PointCloud2Iterator<float> iter_cam_z(pc_rect_cam, "z");
  PointCloud2Iterator<float> iter_img_x(pc_img, "x");
  PointCloud2Iterator<float> iter_img_y(pc_img, "y");
  PointCloud2Iterator<float> iter_img_z(pc_img, "z");

  for (; iter_cam_z != iter_cam_z.end();
       ++iter_cam_z, ++iter_img_x, ++iter_img_y, ++iter_img_z)
  {

    const float& depth = *iter_cam_z;
    if (depth <= 0) continue;

    const float& img_x = *iter_img_x;
    const float& img_y = *iter_img_y;
    const float& img_z = *iter_img_z;

    if (img_z == 0) continue;
    const int u = img_x / img_z;
    const int v = img_y / img_z; 


    if (inImage(cam_info, u, v))
    {
      if (depth_image.at<float>(v, u) == 0  ||
          depth_image.at<float>(v, u) > depth)
      {
        depth_image.at<float>(v, u) = depth;
      }
    }
  }
}

void DepthCompletion::depthImageToPointCloud(
  const cv::Mat depth_image,
  const CameraInfoConstPtr& cam_info,
  PointCloud2 & pc)
{

  pcl::PointCloud<pcl::PointXYZ> pcl_img;
  for(int u = 0; u < depth_image.cols; u++)
  {
    for(int v = 0; v < depth_image.rows; v++)
    {
      const float & depth = depth_image.at<float>(v, u);
      if (depth == 0) continue;
      pcl::PointXYZ point;
      const float img_x = u * depth;
      const float img_y = v * depth;
      const float img_z = depth;
      point.x = img_x;
      point.y = img_y;
      point.z = img_z;
      pcl_img.points.push_back(point);
    }
  }
  PointCloud2 pc_img;
  pcl::toROSMsg(pcl_img, pc_img);

  Eigen::Matrix4f P = Eigen::Matrix4f();
  P << cam_info->P[0], cam_info->P[1], cam_info->P[2], cam_info->P[3],
       cam_info->P[4], cam_info->P[5], cam_info->P[6], cam_info->P[7], 
       cam_info->P[8], cam_info->P[9], cam_info->P[10], cam_info->P[11],
                    0,              0,               0,               1;
  PointCloud2 pc_rect_cam;
  pcl_ros::transformPointCloud(P.inverse(), pc_img, pc_rect_cam);

  PointCloud2 pc_cam;
  Eigen::Matrix4f R = Eigen::Matrix4f();
  R << cam_info->R[0], cam_info->R[1], cam_info->R[2], 0,
       cam_info->R[3], cam_info->R[4], cam_info->R[5], 0,
       cam_info->R[6], cam_info->R[7], cam_info->R[8], 0,
                    0,              0,              0, 1;
  pcl_ros::transformPointCloud(R.inverse(), pc_rect_cam, pc_cam);

  std::string target_frame = pc.header.frame_id;
  std::string source_frame = cam_info->header.frame_id;
  pc_cam.header.stamp = ros::Time::now();
  pc_cam.header.frame_id = source_frame;
  bool transformed = pcl_ros::transformPointCloud(target_frame, pc_cam, pc, listener_);
}

void DepthCompletion::configCallback(sensor_processing::DepthCompletionParamsConfig &config, uint32_t level)
{
  diamond_kernel_size_ = config.diamondKernelSize;
  full_kernel_size_ = convertFullKernelSize(config.fullKernelSize);
  ROS_DEBUG("Reconfigure Request");
  ROS_DEBUG("diamondKernelSize %d", diamond_kernel_size_);
  ROS_DEBUG("fullKernelSize %d", full_kernel_size_);
}

int DepthCompletion::convertFullKernelSize(const int full_kernel_size){
  return 2 * full_kernel_size + 1;
}

void DepthCompletion::processDepthCompletion(
  const CameraInfoConstPtr& cam_info, 
  const cv::Mat depth_image, 
  cv::Mat & depth_completion_image)
{
  cv::Mat inv_depth_image = cv::Mat::zeros(cam_info->width, cam_info->height, CV_32FC1);
  for(int u = 0; u < depth_image.cols; u++)
  {
    for(int v = 0; v < depth_image.rows; v++)
    {
      const float & depth = depth_image.at<float>(v, u);
      if (depth == 0) continue;
        inv_depth_image.at<float>(v, u) = 100 - depth;
    }
  }

  cv::Mat diamond_kernel = 
    cv::Mat::zeros(2 * diamond_kernel_size_ + 1, 2 * diamond_kernel_size_ + 1, CV_8UC1);
  for(int i = -diamond_kernel_size_; i <=diamond_kernel_size_; i++)
  {
    int r = diamond_kernel_size_ - std::abs(i);
    for(int j = -r; j <= r; j++)
    {
      int x = diamond_kernel_size_ + i;
      int y = diamond_kernel_size_ + j;
      diamond_kernel.at<uint8_t>(y, x) = 1;
    }
  }
  cv::Mat final_image;
  cv::dilate(inv_depth_image, final_image, diamond_kernel);

  cv::Mat full_kernel_5 = cv::Mat::ones(5, 5, CV_8UC1);
  cv::morphologyEx(final_image, final_image, cv::MORPH_CLOSE, full_kernel_5);
  
  cv::Mat dilated_depth_image;
  cv::Mat full_kernel_7 = cv::Mat::ones(7, 7, CV_8UC1);
  cv::dilate(final_image, dilated_depth_image, full_kernel_7);

  for(int u = 0; u < final_image.cols; u++)
  {
    for(int v = 0; v < final_image.rows; v++)
    {
      const float & depth = final_image.at<float>(v, u);
      if (depth != 0) continue;
      final_image.at<float>(v, u) = dilated_depth_image.at<float>(v, u);
    }
  }

  cv::medianBlur(final_image, final_image, 5);
  cv::Mat blurred_depth_image = final_image;
  //cv::GaussianBlur(final_image, blurred_depth_image, cv::Size(5, 5), 0);

  depth_completion_image = cv::Mat::zeros(cam_info->width, cam_info->height, CV_32FC1);
  for(int u = 0; u < final_image.cols; u++)
  {
    for(int v = 0; v < final_image.rows; v++)
    {
      const float & depth = final_image.at<float>(v, u);
      if (depth == 0) continue;
      depth_completion_image.at<float>(v, u) = 100 - blurred_depth_image.at<float>(v, u);
    }
  }
}

}

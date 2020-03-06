#ifndef DEPTHCOMPLETION_H
#define DEPTHCOMPLETION_H

#include "sensor_processing/DepthCompletionParamsConfig.h"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>

namespace sensor_processing
{

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

class DepthCompletion
{
 public:
  //! Constructor.
  explicit DepthCompletion(ros::NodeHandle nh);

 private:
  void configCallback(sensor_processing::DepthCompletionParamsConfig &config, uint32_t level);

  bool inImage(const CameraInfoConstPtr& cam_info, const int u, const int v);
  void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img);
  void callback(const PointCloud2ConstPtr& pc_msg,
                const CameraInfoConstPtr& l_info_msg,
                const ImageConstPtr& l_image_msg);

  void processDepthCompletion(
    const CameraInfoConstPtr& cam_info, 
    const cv::Mat depth_image, 
    cv::Mat & depth_completion_image);

  void pointCloudToDepthImage(
    const PointCloud2ConstPtr& pc,
    const CameraInfoConstPtr& cam_info,
    cv::Mat& depth_image);
  void depthImageToPointCloud(
    const cv::Mat depth_image,
    const CameraInfoConstPtr& cam_info,
    PointCloud2 & pc);

  int convertFullKernelSize(const int full_kernel_size);

  
  ros::NodeHandle nh_;

  message_filters::Subscriber<PointCloud2> sub_pointcloud_;
  message_filters::Subscriber<CameraInfo> sub_left_color_camera_info_;
  message_filters::Subscriber<Image> sub_left_color_image_;
  typedef ExactTime<PointCloud2, CameraInfo, Image> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  ExactSync exact_sync_;

  ros::Publisher pub_depth_image_;
  ros::Publisher pub_completion_image_;
  ros::Publisher pub_completed_pointcloud_;

  dynamic_reconfigure::Server<sensor_processing::DepthCompletionParamsConfig> dr_srv_;

  tf::TransformListener listener_;

  int diamond_kernel_size_;
  int full_kernel_size_;
};
}

#endif  // DEPTHCOMPLETION_H

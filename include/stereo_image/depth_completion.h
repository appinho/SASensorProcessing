#ifndef DEPTHCOMPLETION_H
#define DEPTHCOMPLETION_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <stereo_image/DepthCompletionParamsConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>

namespace stereo_image
{

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

class DepthCompletion
{
 public:
  //! Constructor.
  explicit DepthCompletion(ros::NodeHandle nh);

 private:
  void configCallback(stereo_image::DepthCompletionParamsConfig &config, uint32_t level);

  bool inImage(const CameraInfoConstPtr& cam_info, const int u, const int v);
  void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img);
  void callback(const PointCloud2ConstPtr& pc_msg,
                const CameraInfoConstPtr& l_info_msg,
                const ImageConstPtr& l_image_msg);

  void processDepthCompletion();

  
  ros::NodeHandle nh_;

  message_filters::Subscriber<PointCloud2> sub_pointcloud_;
  message_filters::Subscriber<CameraInfo> sub_left_color_camera_info_;
  message_filters::Subscriber<Image> sub_left_color_image_;
  typedef ExactTime<PointCloud2, CameraInfo, Image> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  ExactSync exact_sync_;

  ros::Publisher pub_completed_image_;
  ros::Publisher pub_depth_image_;
  ros::Publisher pub_completed_pointcloud_;

  dynamic_reconfigure::Server<stereo_image::DepthCompletionParamsConfig> dr_srv_;

  tf::TransformListener listener_;
};
}

#endif  // DEPTHCOMPLETION_H

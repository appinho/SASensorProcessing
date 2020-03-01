#ifndef STEREO_H
#define STEREO_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <stereo_image/StereoParamsConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

//#include <opencv2/calib3d/calib3d.hpp>
#include <image_geometry/stereo_camera_model.h>

namespace stereo_image
{

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

class Stereo
{
 public:
  //! Constructor.
  explicit Stereo(ros::NodeHandle nh);

 private:
  void configCallback(stereo_image::StereoParamsConfig &config, uint32_t level);

  void callback(const ImageConstPtr& l_image_msg,
                const CameraInfoConstPtr& l_info_msg,
                const ImageConstPtr& r_image_msg,
                const CameraInfoConstPtr& r_info_msg);

  int convertNumDisparities(const int num_disparities);
  int convertBlockSize(const int block_size);
  
  ros::NodeHandle nh_;

  message_filters::Subscriber<Image> sub_left_image_;
  message_filters::Subscriber<CameraInfo> sub_left_camera_info_;
  message_filters::Subscriber<Image> sub_right_image_;
  message_filters::Subscriber<CameraInfo> sub_right_camera_info_;
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  ExactSync exact_sync_;

  ros::Publisher pub_disparity_image_;

  dynamic_reconfigure::Server<stereo_image::StereoParamsConfig> dr_srv_;

  image_geometry::StereoCameraModel model_;
  cv::Ptr<cv::StereoBM> block_matcher_;
  cv::Ptr<cv::StereoSGBM> sg_block_matcher_;

  cv::Mat_<int16_t> disparity16_;

  int num_disparities_;
  int block_size_;
};
}

#endif  // STEREO_H

#include <stereo_image/stereo.h>

namespace stereo_image
{
Stereo::Stereo(ros::NodeHandle nh) : 
  nh_(nh),
  exact_sync_(ExactPolicy(10),
    sub_left_image_, sub_left_camera_info_,
    sub_right_image_, sub_right_camera_info_)
{
  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server values can be overwritten.
  dynamic_reconfigure::Server<stereo_image::StereoParamsConfig>::CallbackType cb;
  cb = boost::bind(&Stereo::configCallback, this, _1, _2);
  dr_srv_.setCallback(cb);

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  num_disparities_ = convertNumDisparities(2);
  block_size_ = convertBlockSize(8);
  pnh.param("numDisparities", num_disparities_, num_disparities_);
  pnh.param("blockSize", block_size_, block_size_);

  sub_left_image_.subscribe(nh_, "kitti/camera_gray_left/image_raw", 1);
  sub_left_camera_info_.subscribe(nh_, "/kitti/camera_gray_left/camera_info", 1);
  sub_right_image_.subscribe(nh_, "/kitti/camera_gray_right/image_raw", 1);
  sub_right_camera_info_.subscribe(nh_, "/kitti/camera_gray_right/camera_info", 1);
  exact_sync_.registerCallback(boost::bind(&Stereo::callback,
                                              this, _1, _2, _3, _4));
}

void Stereo::callback(const ImageConstPtr& l_image_msg,
                      const CameraInfoConstPtr& l_info_msg,
                      const ImageConstPtr& r_image_msg,
                      const CameraInfoConstPtr& r_info_msg)
{

  ROS_INFO("Callback");
}

void Stereo::configCallback(stereo_image::StereoParamsConfig &config, uint32_t level __attribute__((unused)))
{
  num_disparities_ = convertNumDisparities(config.numDisparities);
  block_size_ = convertBlockSize(config.blockSize);
  ROS_INFO("Reconfigure Request");
  ROS_INFO("numDisparities %d", num_disparities_);
  ROS_INFO("blockSize %d", block_size_);
}

int Stereo::convertNumDisparities(const int num_disparities){
  return 16 * num_disparities;
}

int Stereo::convertBlockSize(const int block_size){
  return 2 * block_size + 5;
}

}

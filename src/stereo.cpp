#include <stereo_image/stereo.h>

namespace stereo_image
{
Stereo::Stereo(ros::NodeHandle nh) : 
  nh_(nh)
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

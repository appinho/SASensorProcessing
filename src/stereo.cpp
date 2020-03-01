#include <stereo_image/stereo.h>
#include <cv_bridge/cv_bridge.h>

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

  pub_disparity_image_ = nh_.advertise<Image>("/kitti/disparity_image", 1);

  block_matcher_ = cv::StereoBM::create();
  sg_block_matcher_ = cv::StereoSGBM::create(1, 1, 10);

}

void Stereo::callback(const ImageConstPtr& l_image_msg,
                      const CameraInfoConstPtr& l_info_msg,
                      const ImageConstPtr& r_image_msg,
                      const CameraInfoConstPtr& r_info_msg)
{

  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);
  ROS_INFO("Callback");
  
  // Allocate new disparity image message
  //DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
  //disp_msg->header         = l_info_msg->header;
  //disp_msg->image.header   = l_info_msg->header;

  // Compute window of (potentially) valid disparities
  //int border   = block_matcher_.getCorrelationWindowSize() / 2;
  //int left   = block_matcher_.getDisparityRange() + block_matcher_.getMinDisparity() + border - 1;
  //int wtf = (block_matcher_.getMinDisparity() >= 0) ? border + block_matcher_.getMinDisparity() : std::max(border, -block_matcher_.getMinDisparity());
  //int right  = disp_msg->image.width - 1 - wtf;
  //int top    = border;
  //int bottom = disp_msg->image.height - 1 - border;
  //disp_msg->valid_window.x_offset = left;
  //disp_msg->valid_window.y_offset = top;
  //disp_msg->valid_window.width    = right - left;
  //disp_msg->valid_window.height   = bottom - top;

  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
  const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;

  // Perform block matching to find the disparities
  block_matcher_->compute(l_image, r_image, disparity16_);

  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  double cx_l = model_.left().cx();
  double cx_r = model_.right().cx();
  /*
  if (cx_l != cx_r) {
    cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                              reinterpret_cast<float*>(&disp_msg->image.data[0]),
                              disp_msg->image.step);
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
  }
  */
  cv_bridge::CvImage cv_bridge_disparity_image;
  cv_bridge_disparity_image.image = disparity16_;
  cv_bridge_disparity_image.encoding = "mono16";
  cv_bridge_disparity_image.header.stamp = l_image_msg->header.stamp;
  pub_disparity_image_.publish(cv_bridge_disparity_image.toImageMsg());
  
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

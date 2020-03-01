#ifndef STEREO_H
#define STEREO_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <stereo_image/StereoParamsConfig.h>

namespace stereo_image
{
class Stereo
{
 public:
  //! Constructor.
  explicit Stereo(ros::NodeHandle nh);

 private:
  void configCallback(stereo_image::StereoParamsConfig &config, uint32_t level);

  ros::NodeHandle nh_;

  ros::Publisher pub_;

  dynamic_reconfigure::Server<stereo_image::StereoParamsConfig> dr_srv_;


  int convertNumDisparities(const int num_disparities);
  int num_disparities_;
  int convertBlockSize(const int block_size);
  int block_size_;
};
}

#endif  // STEREO_H

#include "sensor_processing/stereo_vision.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "stereo_vision");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  sensor_processing::StereoVision node(nh);

  // Let ROS handle all callbacks.
  ros::spin();

  return 0;
}  // end main()
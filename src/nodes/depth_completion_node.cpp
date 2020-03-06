#include "sensor_processing/depth_completion.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "depth_completion");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  sensor_processing::DepthCompletion node(nh);

  // Let ROS handle all callbacks.
  ros::spin();

  return 0;
}  // end main()
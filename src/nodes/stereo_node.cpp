#include <stereo_image/stereo.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "stereo");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  stereo_image::Stereo node(nh);

  // Let ROS handle all callbacks.
  ros::spin();

  return 0;
}  // end main()
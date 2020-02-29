cmake_minimum_required(VERSION 2.8.3)
project(stereo_image)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation rosgraph_msgs rospy std_msgs message_filters)

generate_dynamic_reconfigure_options(
  cfg/stereo_params.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime rosgraph_msgs rospy std_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

endif()


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS src/stereo_image_node.py src/pytalker.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

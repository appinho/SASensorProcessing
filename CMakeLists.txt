cmake_minimum_required(VERSION 2.8.3)
project(sensor_processing)

find_package(catkin REQUIRED 
  dynamic_reconfigure
  message_generation
  rosgraph_msgs
  rospy
  roscpp
  std_msgs
  message_filters 
  image_geometry
  cv_bridge
  pcl_ros
)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

generate_dynamic_reconfigure_options(
  cfg/stereo_vision_params.cfg
  cfg/depth_completion_params.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime rosgraph_msgs rospy roscpp std_msgs image_geometry
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(stereo src/nodes/stereo_vision_node.cpp src/stereo_vision.cpp)
target_link_libraries(stereo ${catkin_LIBRARIES})
add_dependencies(stereo stereo_image_gencfg)

add_executable(depth_completion src/nodes/depth_completion_node.cpp src/depth_completion.cpp)
target_link_libraries(depth_completion ${catkin_LIBRARIES})
add_dependencies(depth_completion depth_completion_gencfg)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

endif()


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS src/pystereo.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS stereo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

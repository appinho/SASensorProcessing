cmake_minimum_required(VERSION 2.8.3)
project(stereo_image)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation rosgraph_msgs rospy std_msgs message_filters)

generate_dynamic_reconfigure_options(
  cfg/stereo_params.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime rosgraph_msgs rospy std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(stereo src/nodes/stereo_node.cpp src/stereo.cpp)
target_link_libraries(stereo ${catkin_LIBRARIES})
add_dependencies(stereo stereo_image_gencfg )

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

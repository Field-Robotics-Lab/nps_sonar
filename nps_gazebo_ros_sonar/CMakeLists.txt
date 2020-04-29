cmake_minimum_required(VERSION 2.8.3)
project(nps_sonar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  cv_bridge
  image_transport
  tf
)

find_package(gazebo REQUIRED)
find_package(OpenCV)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
LIBRARIES
INCLUDE_DIRS
DEPENDS
)

## Plugins
add_library(nps_gazebo_ros_sonar src/nps_gazebo_ros_sonar.cpp)
target_link_libraries(nps_gazebo_ros_sonar RayPlugin ${catkin_LIBRARIES})

#target_link_libraries(gtri_based_fls_plugin
#  cloud_to_image color_maps
#  ${catkin_LIBRARIES} 
#  ${OpenCV_LIBRARIES}
#  ${GAZEBO_LIBRARIES}
#  )

# for launch
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)


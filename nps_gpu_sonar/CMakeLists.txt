cmake_minimum_required(VERSION 2.8.3)
project(nps_gpu_sonar)

find_package(catkin REQUIRED tf)

find_package(gazebo REQUIRED)
find_package(OpenCV)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package()

### gpu_sonar rendering component
add_library(GpuSonarRendering
            rendering/GpuSonar.cc
            rendering/GpuSonar.hh
            rendering/GpuSonarPrivate.hh
            rendering/GpuSonarDataIterator.hh
            rendering/GpuSonarDataIteratorImpl.hh)
target_link_libraries(GpuSonarRendering ${GAZEBO_LIBRARIES})

### gpu_sonar Gazebo world component with the sensor that it loads
add_library(GpuSonarSensor SHARED
            gazebo/add_gpu_sonar_sensor.cc
            gazebo/GpuSonarSensor.cc
            gazebo/GpuSonarSensorTypes.hh
            gazebo/GpuSonarSensor.hh
            gazebo/GpuSonarSensorPrivate.hh)
target_link_libraries(GpuSonarSensor GpuSonarRendering ${GAZEBO_LIBRARIES})

### gpu_sonar ROS component
# new gpu_sonar based Gazebo ROS sonar plugin
add_library(gazebo_ros_gpu_sonar_plugin
            gazebo/GpuSonarPlugin.cc
            gazebo/GpuSonarPlugin.hh
            gazebo/GpuSonarSensor.hh
            ros/gazebo_ros_gpu_sonar.h
            ros/gazebo_ros_gpu_sonar.cpp)
target_link_libraries(gazebo_ros_gpu_sonar_plugin
                      ${catkin_LIBRARIES})

# for launch
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)


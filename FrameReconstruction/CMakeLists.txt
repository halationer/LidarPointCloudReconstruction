cmake_minimum_required(VERSION 2.8.3)
project(frame_reconstruction)

## Compile as C++11
SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++11)
# supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES record_odomlaser
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)




include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  src
)


###### build ########
add_executable(frame_reconstruction src/FrameRecon.cpp src/main.cpp)

target_link_libraries(frame_reconstruction ${catkin_LIBRARIES} ${PCL_LIBRARIES})
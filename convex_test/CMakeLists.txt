cmake_minimum_required(VERSION 3.0.2)
project(convex_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(CGAL QUIET COMPONENTS)
include( ${CGAL_USE_FILE} )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES record_odomlaser
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src DIR_SRCS)

###### build ########
add_executable(convex_test ${DIR_SRCS})

target_link_libraries(convex_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_to_cached_list( CGAL_EXECUTABLE_TARGETS convex_test )
target_link_libraries(convex_test ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
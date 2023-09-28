
cmake_minimum_required(VERSION 2.8.3)
project(simple_frame)

## Compile as C++17
SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++17)
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
             sensor_msgs
             filters
             fusion_msgs
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
    #${OCTOMAP_INCLUDE_DIRS}
)


aux_source_directory(src DIR_SRCS)

###### build ########
add_executable(simple_frame ${DIR_SRCS})

target_link_libraries(simple_frame ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_to_cached_list( CGAL_EXECUTABLE_TARGETS simple_frame )
target_link_libraries(simple_frame ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
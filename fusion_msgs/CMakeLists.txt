
cmake_minimum_required(VERSION 2.8.3)
project(fusion_msgs)

find_package(catkin REQUIRED COMPONENTS
             message_generation
             shape_msgs
)

add_message_files(
	FILES
	MeshArray.msg
)

generate_messages(
	DEPENDENCIES
    shape_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES record_odomlaser
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
    CATKIN_DEPENDS message_runtime
)


include_directories(
	include
    ${catkin_INCLUDE_DIRS}
    #${OCTOMAP_INCLUDE_DIRS}
)
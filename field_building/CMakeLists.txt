cmake_minimum_required(VERSION 3.0.2)
project(field_building)

## Compile as C++11, supported in ROS Kinetic and newer
##add_compile_options(-std=c++2a)
set(CMAKE_CXX_STANDARD 20)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf2_ros
	pcl_ros
	tf_conversions
	geometry_msgs
	gps_common
	octomap_ros
)

## System dependencies are found with CMake's conventions
find_package(glog)



###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf2_ros pcl_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS})

add_executable(cloud_aggregator
	src/cloud_aggregator_node.cpp)
target_link_libraries(cloud_aggregator
	${catkin_LIBRARIES}
	glog
)

add_executable(utm_odometry_node src/utm_odometry_node.cpp)
set_target_properties(utm_odometry_node PROPERTIES OUTPUT_NAME "utm_odometry_node")
target_link_libraries(utm_odometry_node ${catkin_LIBRARIES})

IF(${USE_OPEN3D})
	find_package(Open3D REQUIRED)
	add_executable(open3d_cloud_aggregator
		src/opend3d_cloud_aggregator.cpp)
	target_link_libraries(open3d_cloud_aggregator
		${catkin_LIBRARIES}
		glog
		${Open3D_LIBRARIES}
	)
ENDIF()


#############
## Install ##
#############

#############
## Testing ##
#############


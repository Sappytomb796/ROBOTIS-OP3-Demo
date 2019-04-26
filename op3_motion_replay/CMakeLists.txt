cmake_minimum_required(VERSION 2.8.3)

# name package
project(op3_motion_replay)

# ROS Kinect and new supports c++11
add_compile_options(-std=c++11)

# find other CMake packages required to build project
find_package(catkin REQUIRED COMPONENTS
	robotis_controller_msgs
	roscpp
	sensor_msgs
	std_msgs
)

# CMake macro to specify catkin info
# exported path to .h files
catkin_package(
	INCLUDE_DIRS include
)

# directories to include
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# specify executable target to build
add_executable(
	${PROJECT_NAME}_node
	src/op3_motion_replay.cpp
	src/op3_motion_replay_node.cpp
)

# target dependencies
add_dependencies(
	${PROJECT_NAME}_node
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

# specify which libraries an executable target links against
target_link_libraries(
	${PROJECT_NAME}_node
	${catkin_LIBRARIES}
)
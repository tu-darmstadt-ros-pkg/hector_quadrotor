cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

###########
## Build ##
###########

rosbuild_add_library(hector_quadrotor_pose_estimation_node
  src/pose_estimation_node.cpp
)

rosbuild_add_library(hector_quadrotor_pose_estimation_nodelet
  src/pose_estimation_nodelet.cpp
)
target_link_libraries(hector_quadrotor_pose_estimation_nodelet hector_quadrotor_pose_estimation_node)

rosbuild_add_executable(hector_quadrotor_pose_estimation
  src/main.cpp
)
target_link_libraries(hector_quadrotor_pose_estimation hector_quadrotor_pose_estimation_node)

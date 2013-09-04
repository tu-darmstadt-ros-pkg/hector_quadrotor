cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})

rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_add_boost_directories()

rosbuild_add_library(hector_gazebo_ros_baro src/gazebo_ros_baro.cpp)
#rosbuild_link_boost(hector_gazebo_ros_baro thread)
target_link_libraries(hector_gazebo_ros_baro ${GAZEBO_LIBRARIES})

rosbuild_add_library(hector_gazebo_quadrotor_simple_controller src/gazebo_quadrotor_simple_controller.cpp)
#rosbuild_link_boost(hector_gazebo_quadrotor_simple_controller thread)
target_link_libraries(hector_gazebo_quadrotor_simple_controller ${GAZEBO_LIBRARIES})

rosbuild_find_ros_package(hector_quadrotor_model)

find_library(hector_quadrotor_propulsion_LIBRARY
  NAMES libhector_quadrotor_propulsion.so
  HINTS ${hector_quadrotor_model_PACKAGE_PATH}/lib
)
if(hector_quadrotor_propulsion_LIBRARY)
  rosbuild_add_library(hector_gazebo_quadrotor_propulsion src/gazebo_quadrotor_propulsion.cpp)
  target_link_libraries(hector_gazebo_quadrotor_propulsion ${GAZEBO_LIBRARIES} ${hector_quadrotor_propulsion_LIBRARY})
else()
  message(WARNING "Quadrotor propulsion model is not available. Skipping target hector_gazebo_quadrotor_propulsion...")
endif()

find_library(hector_quadrotor_aerodynamics_LIBRARY
  NAMES libhector_quadrotor_aerodynamics.so
  HINTS ${hector_quadrotor_model_PACKAGE_PATH}/lib
)
if(hector_quadrotor_aerodynamics_LIBRARY)
  rosbuild_add_library(hector_gazebo_quadrotor_aerodynamics src/gazebo_quadrotor_aerodynamics.cpp)
  target_link_libraries(hector_gazebo_quadrotor_aerodynamics ${GAZEBO_LIBRARIES} ${hector_quadrotor_aerodynamics_LIBRARY})
else()
  message(WARNING "Quadrotor aerodynamics model is not available. Skipping target hector_gazebo_quadrotor_aerodynamics...")
endif()

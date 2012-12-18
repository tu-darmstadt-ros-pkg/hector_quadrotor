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
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_add_boost_directories()

include_directories(matlab/codegen/lib)
add_subdirectory(matlab)

option(BUILD_PROPULSION_MODEL "Build a model of the quadrotor propulsion system (requires Matlab)" ON)
if(BUILD_PROPULSION_MODEL)
  if(TARGET quadrotor_model)
    rosbuild_add_library(hector_gazebo_quadrotor_propulsion src/quadrotor_propulsion.cpp)
    target_link_libraries(hector_gazebo_quadrotor_propulsion ${CMAKE_CURRENT_BINARY_DIR}/matlab/quadrotorPropulsion.a)
    add_dependencies(hector_gazebo_quadrotor_propulsion quadrotor_model)
  else()
    message(WARNING "Quadrotor model is not available. Skipping target hector_gazebo_quadrotor_propulsion...")
    set(BUILD_PROPULSION_MODEL OFF)
  endif()
endif()

option(BUILD_AERODYNAMICS_MODEL "Use a model of the quadrotor aerodynamics (requires Matlab)" ON)
if(BUILD_PROPULSION_MODEL)
  if(TARGET quadrotor_model)
    rosbuild_add_library(hector_gazebo_quadrotor_aerodynamics src/quadrotor_aerodynamics.cpp)
    target_link_libraries(hector_gazebo_quadrotor_aerodynamics ${CMAKE_CURRENT_BINARY_DIR}/matlab/quadrotorDrag.a)
    add_dependencies(hector_gazebo_quadrotor_aerodynamics quadrotor_model)
  else()
    message(WARNING "Quadrotor model is not available. Skipping target hector_gazebo_quadrotor_aerodynamics...")
    set(BUILD_AERODYNAMICS_MODEL OFF)
  endif()
endif()

rosbuild_add_executable(test_trajectory src/test_trajectory.cpp)


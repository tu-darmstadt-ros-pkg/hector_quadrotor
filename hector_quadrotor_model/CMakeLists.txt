cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs hector_uav_msgs roscpp)
include_directories(include ${catkin_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

# Find Eigen3 headers
# see http://wiki.ros.org/jade/Migration#Eigen_CMake_Module_in_cmake_modules
# and https://github.com/ros/cmake_modules/issues/35
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_FOUND ${EIGEN_FOUND})
  set(EIGEN3_INCLUDE_DIR ${EIGEN_INCLUDE_DIRS})
  # set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  # Possibly map additional variables to the EIGEN3_ prefix.
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES hector_quadrotor_propulsion hector_quadrotor_aerodynamics
    CATKIN_DEPENDS geometry_msgs hector_uav_msgs roscpp
    DEPENDS Boost
)

###########
## Build ##
###########

add_library(hector_quadrotor_propulsion src/quadrotor_propulsion.cpp)
add_dependencies(hector_quadrotor_propulsion hector_uav_msgs_generate_messages_cpp)
target_link_libraries(hector_quadrotor_propulsion ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(hector_quadrotor_propulsion quadrotorPropulsion)

add_library(hector_quadrotor_aerodynamics src/quadrotor_aerodynamics.cpp)
target_link_libraries(hector_quadrotor_aerodynamics ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(hector_quadrotor_aerodynamics quadrotorDrag)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(
  TARGETS hector_quadrotor_propulsion hector_quadrotor_aerodynamics
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY param DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


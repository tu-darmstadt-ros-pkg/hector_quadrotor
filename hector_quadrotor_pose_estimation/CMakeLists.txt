cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_pose_estimation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS hector_pose_estimation hector_uav_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   hector_uav_msgs
# )

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
    LIBRARIES quadrotor_pose_estimation_nodelet quadrotor_pose_estimation_node
    CATKIN_DEPENDS hector_pose_estimation hector_uav_msgs
    DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(hector_quadrotor_pose_estimation_node
  src/pose_estimation_node.cpp
)
target_link_libraries(hector_quadrotor_pose_estimation_node ${catkin_LIBRARIES})
add_dependencies(hector_quadrotor_pose_estimation_node hector_uav_msgs_generate_messages_cpp)

add_library(hector_quadrotor_pose_estimation_nodelet
  src/pose_estimation_nodelet.cpp
)
target_link_libraries(hector_quadrotor_pose_estimation_nodelet hector_quadrotor_pose_estimation_node ${catkin_LIBRARIES})

add_executable(hector_quadrotor_pose_estimation
  src/main.cpp
)
target_link_libraries(hector_quadrotor_pose_estimation hector_quadrotor_pose_estimation_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS hector_quadrotor_pose_estimation_nodelet hector_quadrotor_pose_estimation_node hector_quadrotor_pose_estimation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  hector_quadrotor_pose_estimation_nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

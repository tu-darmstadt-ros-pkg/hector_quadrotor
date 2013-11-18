cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs nav_msgs hector_uav_msgs hardware_interface controller_interface)
include_directories(include ${catkin_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
    LIBRARIES quadrotor_controller
    CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs hector_uav_msgs hardware_interface controller_interface
    DEPENDS
)

###########
## Build ##
###########

add_library(quadrotor_controller src/quadrotor_interface.cpp src/pid.cpp)
target_link_libraries(quadrotor_controller ${catkin_LIBRARIES})
add_dependencies(quadrotor_controller hector_uav_msgs_generate_messages_cpp)

add_library(pose_controller src/pose_controller.cpp)
target_link_libraries(pose_controller quadrotor_controller)

add_library(twist_controller src/twist_controller.cpp)
target_link_libraries(twist_controller quadrotor_controller)

#add_library(aerodynamics_plugin src/aerodynamics_plugin.cpp)
#target_link_libraries(aerodynamics_plugin quadrotor_controller)

#add_library(propulsion_plugin src/propulsion_plugin.cpp)
#target_link_libraries(propulsion_plugin quadrotor_controller)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS quadrotor_controller pose_controller twist_controller #aerodynamics_plugin propulsion_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

######################
## Additional tools ##
######################

add_executable(test_trajectory src/test_trajectory.cpp)
target_link_libraries(test_trajectory ${catkin_LIBRARIES})

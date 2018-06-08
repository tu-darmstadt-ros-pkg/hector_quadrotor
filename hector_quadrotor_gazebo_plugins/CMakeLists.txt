cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_gazebo_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp gazebo_dev hector_gazebo_plugins hector_quadrotor_model geometry_msgs hector_uav_msgs dynamic_reconfigure)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find hector_quadrotor_model optional libraries
if(TARGET hector_quadrotor_propulsion)
  set(hector_quadrotor_propulsion_LIBRARY hector_quadrotor_propulsion)
else()
  find_library(hector_quadrotor_propulsion_LIBRARY hector_quadrotor_propulsion)
endif()
if(TARGET hector_quadrotor_propulsion)
  set(hector_quadrotor_aerodynamics_LIBRARY hector_quadrotor_aerodynamics)
else()
  find_library(hector_quadrotor_aerodynamics_LIBRARY hector_quadrotor_aerodynamics)
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS thread)
# include_directories(${Boost_INCLUDE_DIRS})

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
    LIBRARIES
    CATKIN_DEPENDS roscpp hector_gazebo_plugins hector_quadrotor_model geometry_msgs hector_uav_msgs dynamic_reconfigure
)

###########
## Build ##
###########

add_library(hector_gazebo_ros_baro src/gazebo_ros_baro.cpp)
target_link_libraries(hector_gazebo_ros_baro ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hector_gazebo_ros_baro hector_uav_msgs_generate_messages_cpp)

add_library(hector_gazebo_quadrotor_simple_controller src/gazebo_quadrotor_simple_controller.cpp)
target_link_libraries(hector_gazebo_quadrotor_simple_controller ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

if(hector_quadrotor_propulsion_LIBRARY)
  add_library(hector_gazebo_quadrotor_propulsion src/gazebo_quadrotor_propulsion.cpp)
  target_link_libraries(hector_gazebo_quadrotor_propulsion ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${hector_quadrotor_propulsion_LIBRARY})
  add_dependencies(hector_gazebo_quadrotor_propulsion hector_uav_msgs_generate_messages_cpp)
else()
  message(WARNING "Quadrotor propulsion model is not available. Skipping target hector_gazebo_quadrotor_propulsion...")
endif()

if(hector_quadrotor_aerodynamics_LIBRARY)
  add_library(hector_gazebo_quadrotor_aerodynamics src/gazebo_quadrotor_aerodynamics.cpp)
  target_link_libraries(hector_gazebo_quadrotor_aerodynamics ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${hector_quadrotor_aerodynamics_LIBRARY})
else()
  message(WARNING "Quadrotor aerodynamics model is not available. Skipping target hector_gazebo_quadrotor_aerodynamics...")
endif()

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
install(TARGETS hector_gazebo_ros_baro hector_gazebo_quadrotor_simple_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(TARGET hector_gazebo_quadrotor_propulsion)
  install(TARGETS hector_gazebo_quadrotor_propulsion LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()

if(TARGET hector_gazebo_quadrotor_aerodynamics)
  install(TARGETS hector_gazebo_quadrotor_aerodynamics LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


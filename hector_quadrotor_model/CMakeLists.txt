cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs hector_uav_msgs)
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
    LIBRARIES #hector_quadrotor_propulsion hector_quadrotor_aerodynamics
    CATKIN_DEPENDS geometry_msgs hector_uav_msgs
    DEPENDS 
)

###########
## Build ##
###########

include_directories(matlab/codegen/lib)
add_subdirectory(matlab)

option(BUILD_PROPULSION_MODEL "Build a model of the quadrotor propulsion system (requires Matlab)" ON)
if(BUILD_PROPULSION_MODEL)
  if(TARGET quadrotorPropulsion)
    add_library(hector_quadrotor_propulsion src/quadrotor_propulsion.cpp)
    add_dependencies(hector_quadrotor_propulsion hector_uav_msgs_generate_messages_cpp)
    target_link_libraries(hector_quadrotor_propulsion quadrotorPropulsion)
  else()
    message(WARNING "Quadrotor model is not available. Skipping target hector_quadrotor_propulsion...")
    set(BUILD_PROPULSION_MODEL OFF)
  endif()
endif()

option(BUILD_AERODYNAMICS_MODEL "Use a model of the quadrotor aerodynamics (requires Matlab)" ON)
if(BUILD_AERODYNAMICS_MODEL)
  if(TARGET quadrotorDrag)
    add_library(hector_quadrotor_aerodynamics src/quadrotor_aerodynamics.cpp)
    target_link_libraries(hector_quadrotor_aerodynamics quadrotorDrag)
  else()
    message(WARNING "Quadrotor model is not available. Skipping target hector_quadrotor_aerodynamics...")
    set(BUILD_AERODYNAMICS_MODEL OFF)
  endif()
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
# install(TARGETS @{name} @{name}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

if(TARGET hector_quadrotor_propulsion)
  install(TARGETS hector_quadrotor_propulsion LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()

if(TARGET hector_quadrotor_aerodynamics)
  install(TARGETS hector_quadrotor_aerodynamics LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
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

install(DIRECTORY param DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


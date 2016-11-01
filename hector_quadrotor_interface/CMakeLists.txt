cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_interface)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs nav_msgs hector_uav_msgs std_srvs hardware_interface controller_interface tf2 tf2_geometry_msgs urdf)
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES hector_quadrotor_interface
    CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs hector_uav_msgs std_srvs hardware_interface controller_interface
    DEPENDS
)

add_library(hector_quadrotor_interface src/quadrotor_interface.cpp src/helpers.cpp)
target_link_libraries(hector_quadrotor_interface ${catkin_LIBRARIES})
add_dependencies(hector_quadrotor_interface hector_uav_msgs_generate_messages_cpp)

install(TARGETS hector_quadrotor_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

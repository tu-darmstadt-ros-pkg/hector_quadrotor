cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_controller_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_ros_control
  hector_quadrotor_interface
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_quadrotor_controller_gazebo
  CATKIN_DEPENDS gazebo_ros_control hector_quadrotor_interface
)

add_library(hector_quadrotor_controller_gazebo
  src/quadrotor_hardware_gazebo.cpp
)
target_link_libraries(hector_quadrotor_controller_gazebo
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)

install(TARGETS hector_quadrotor_controller_gazebo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  quadrotor_controller_gazebo.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

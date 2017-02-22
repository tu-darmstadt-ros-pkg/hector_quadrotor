cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_controllers)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  geometry_msgs
  hector_quadrotor_interface
  hector_uav_msgs
  roscpp
  tf
  tf2_geometry_msgs
  visualization_msgs
  )
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    LIBRARIES hector_quadrotor_controllers
    CATKIN_DEPENDS roscpp
    DEPENDS
)

add_library(hector_quadrotor_controllers
  src/attitude_controller.cpp
  src/velocity_controller.cpp
  src/position_controller.cpp
  )
target_link_libraries(hector_quadrotor_controllers ${catkin_LIBRARIES})
add_dependencies(hector_quadrotor_controllers ${catkin_EXPORTED_TARGETS})

install(TARGETS hector_quadrotor_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_actions)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  hector_quadrotor_interface
  hector_uav_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  )
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS roscpp
    DEPENDS
)

add_executable(landing_action
  src/landing_action.cpp
  )
target_link_libraries(landing_action ${catkin_LIBRARIES})
add_dependencies(landing_action ${catkin_EXPORTED_TARGETS})

add_executable(pose_action
  src/pose_action.cpp
  )
target_link_libraries(pose_action ${catkin_LIBRARIES})
add_dependencies(pose_action ${catkin_EXPORTED_TARGETS})

add_executable(takeoff_action
  src/takeoff_action.cpp
  )
target_link_libraries(takeoff_action ${catkin_LIBRARIES})
add_dependencies(takeoff_action ${catkin_EXPORTED_TARGETS})

install(TARGETS landing_action pose_action takeoff_action
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

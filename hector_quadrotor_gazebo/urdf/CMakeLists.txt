option(USE_GROUND_TRUTH_FOR_CONTROL "Use the ground truth state as input for the controller" ON)
if(USE_GROUND_TRUTH_FOR_CONTROL)
  set(CONTROL_STATE_TOPIC "ground_truth/state")
  set(CONTROL_IMU_TOPIC "raw_imu")
else()
  set(CONTROL_STATE_TOPIC "state")
  set(CONTROL_IMU_TOPIC "imu")
endif()

option(USE_POSE_ESTIMATION "Use hector_pose_estimation to estimate the pose of the quadrotor." ON)
if(USE_POSE_ESTIMATION)
  rosbuild_find_ros_package(gazebo_rtt_plugin)
  rosbuild_find_ros_package(hector_pose_estimation_rtt)

  if(NOT EXISTS ${gazebo_rtt_plugin_PACKAGE_PATH}/lib/libgazebo_rtt_plugin.so OR
     NOT EXISTS ${hector_pose_estimation_rtt_PACKAGE_PATH}/lib/orocos/gnulinux/libhector_pose_estimation_rtt-gnulinux.so)
    message(STATUS "Not using the pose estimation plugin as the required plugins have not been found. Try to run 'rosmake gazebo_rtt_plugin hector_pose_estimation_rtt'.")
    set(USE_POSE_ESTIMATION OFF)
  endif()
endif()

if(USE_POSE_ESTIMATION)
  set(URDF "${URDF}    <quadrotor_pose_estimation/>\n")
endif()

option(USE_PROPULSION_MODEL "Use a model of the quadrotor propulsion system" OFF)
if(USE_PROPULSION_MODEL)
  rosbuild_find_ros_package(gazebo_rtt_plugin)
  rosbuild_find_ros_package(hector_quadrotor_controller)

  if(NOT EXISTS ${gazebo_rtt_plugin_PACKAGE_PATH}/lib/libgazebo_rtt_plugin.so OR
     NOT EXISTS ${hector_quadrotor_controller_PACKAGE_PATH}/lib/libhector_gazebo_quadrotor_propulsion.so)
    message(STATUS "Not using the propulsion model as the required plugins have not been found. Try to run 'rosmake hector_quadrotor_controller'.")
    set(USE_PROPULSION_MODEL OFF)
  endif()
endif()

if(USE_PROPULSION_MODEL)
  set(URDF "${URDF}    <quadrotor_controller state_topic=\"${CONTROL_STATE_TOPIC}\" imu_topic=\"${CONTROL_IMU_TOPIC}\"/>\n")
  set(URDF "${URDF}    <quadrotor_propulsion/>\n")
else()
  set(URDF "${URDF}    <quadrotor_simple_controller state_topic=\"${CONTROL_STATE_TOPIC}\" imu_topic=\"${CONTROL_IMU_TOPIC}\" />\n")
endif()

option(USE_AERODYNAMICS_MODEL "Use a model of the quadrotor aerodynamics" OFF)
if(USE_AERODYNAMICS_MODEL AND NOT EXISTS ${hector_quadrotor_controller_PACKAGE_PATH}/lib/libhector_gazebo_quadrotor_aerodynamics.so)
  message(STATUS "Not using the aerodynamics model as the required plugin has not been found. Try to run 'rosmake hector_quadrotor_controller'.")
  set(USE_AERODYNAMICS_MODEL OFF)
endif()

if(USE_AERODYNAMICS_MODEL)
  set(URDF "${URDF}    <quadrotor_aerodynamics/>\n")
endif()

configure_file(quadrotor_plugins.urdf.xacro.in ${PROJECT_SOURCE_DIR}/urdf/quadrotor_plugins.urdf.xacro @ONLY)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/urdf/quadrotor_plugins.urdf.xacro)

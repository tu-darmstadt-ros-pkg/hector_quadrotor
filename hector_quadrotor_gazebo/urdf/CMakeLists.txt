# This CMakeLists.txt configures the plugins used for the quadrotor model.
set(MODEL_PLUGINS)

# configuration
option(USE_RTT_CONTROLLER "Use a quadrotor controller implemented as Orocos RTT component to control the quadrotor." OFF)
option(USE_POSE_ESTIMATION "Use hector_pose_estimation to estimate the pose of the quadrotor." OFF)
option(USE_PROPULSION_PLUGIN "Use a model of the quadrotor propulsion system" OFF)
option(USE_AERODYNAMICS_PLUGIN "Use a model of the quadrotor aerodynamics" ON)

# sensor plugins
list(APPEND MODEL_PLUGINS quadrotor_sensors)

# pose estimation plugin
if(USE_POSE_ESTIMATION)
  find_library(gazebo_rtt_plugin_LIBRARY libgazebo_rtt_plugin.so)
  find_library(rtt_hector_pose_estimation_LIBRARY librtt_hector_pose_estimation-gnulinux.so)

  if((NOT TARGET gazebo_rtt_plugin AND NOT gazebo_rtt_plugin_LIBRARY) OR
     (NOT TARGET rtt_hector_pose_estimation AND NOT rtt_hector_pose_estimation_LIBRARY))
    message(SEND_ERROR "Cannot use the pose estimation plugin as the required plugins have not been found. Try to run 'rosmake gazebo_rtt_plugin rtt_hector_pose_estimation' first.")
    set(USE_POSE_ESTIMATION OFF)
  endif()
endif()

if(USE_POSE_ESTIMATION)
  list(APPEND MODEL_PLUGINS quadrotor_rtt_pose_estimation)
  set(CONTROL_STATE_TOPIC "state")
  set(CONTROL_IMU_TOPIC "imu")
else()
  set(CONTROL_STATE_TOPIC "ground_truth/state")
  set(CONTROL_IMU_TOPIC "raw_imu")
endif()

# controller plugin
if(USE_RTT_CONTROLLER)
  list(APPEND MODEL_PLUGINS "quadrotor_rtt_controller state_topic=\"${CONTROL_STATE_TOPIC}\" imu_topic=\"${CONTROL_IMU_TOPIC}\"")
else()
  list(APPEND MODEL_PLUGINS quadrotor_controller)
endif()

# propulsion plugin
if(USE_PROPULSION_PLUGIN)
  find_library(hector_gazebo_quadrotor_propulsion_LIBRARY libhector_gazebo_quadrotor_propulsion.so)

  if(NOT TARGET hector_gazebo_quadrotor_propulsion AND NOT hector_gazebo_quadrotor_propulsion_LIBRARY)
    message(SEND_ERROR "Cannot use the propulsion model as the required plugins have not been found. Try to run 'rosmake hector_quadrotor_gazebo_plugins' first.")
    set(USE_PROPULSION_PLUGIN OFF)
  endif()
endif()

if(USE_PROPULSION_PLUGIN)
  list(APPEND MODEL_PLUGINS quadrotor_propulsion)
else()
  #list(APPEND MODEL_PLUGINS "quadrotor_simple_controller state_topic=\"${CONTROL_STATE_TOPIC}\" imu_topic=\"${CONTROL_IMU_TOPIC}\"")
endif()

# aerodynamics plugin
if(USE_AERODYNAMICS_PLUGIN)
  find_library(hector_gazebo_quadrotor_aerodynamics_LIBRARY libhector_gazebo_quadrotor_aerodynamics.so)

  if(NOT TARGET hector_gazebo_quadrotor_aerodynamics AND NOT hector_gazebo_quadrotor_aerodynamics_LIBRARY)
    message(SEND_ERROR "Cannot use the aerodynamics model as the required plugin has not been found. Try to run 'rosmake hector_quadrotor_gazebo_plugins' first.")
    set(USE_AERODYNAMICS_PLUGIN OFF)
  endif()
endif()

if(USE_AERODYNAMICS_PLUGIN)
  list(APPEND MODEL_PLUGINS quadrotor_aerodynamics)
endif()

# write urdf
set(MODEL_PLUGINS_URDF)
foreach(PLUGIN ${MODEL_PLUGINS})
  set(MODEL_PLUGINS_URDF "${MODEL_PLUGINS_URDF}  <xacro:${PLUGIN} />\n")
endforeach()
configure_file(quadrotor_plugins.gazebo.xacro.in ${PROJECT_SOURCE_DIR}/urdf/quadrotor_plugins.gazebo.xacro @ONLY)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/urdf/quadrotor_plugins.gazebo.xacro)

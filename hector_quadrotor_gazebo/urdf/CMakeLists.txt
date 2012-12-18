rosbuild_find_ros_package(hector_quadrotor_controller)

option(USE_PROPULSION_MODEL "Use a model of the quadrotor propulsion system" ON)
if(USE_PROPULSION_MODEL AND NOT EXISTS ${hector_quadrotor_controller_PACKAGE_PATH}/lib/libhector_gazebo_quadrotor_propulsion.so)
  message(STATUS "Not using the propulsion model as the required plugin is not built. Try running rosmake hector_quadrotor_controller.")
  set(USE_PROPULSION_MODEL OFF)
endif()

if(USE_PROPULSION_MODEL)
  set(URDF "${URDF}    <quadrotor_propulsion/>\n")
else()
  set(URDF "${URDF}    <quadrotor_simple_controller/>\n")
endif()

option(USE_AERODYNAMICS_MODEL "Use a model of the quadrotor aerodynamics" ON)
if(USE_AERODYNAMICS_MODEL AND NOT EXISTS ${hector_quadrotor_controller_PACKAGE_PATH}/lib/libhector_gazebo_quadrotor_aerodynamics.so)
  message(STATUS "Not using the aerodynamics model as the required plugin is not built. Try running rosmake hector_quadrotor_controller.")
  set(USE_AERODYNAMICS_MODEL OFF)
endif()

if(USE_AERODYNAMICS_MODEL)
  set(URDF "${URDF}    <quadrotor_aerodynamics/>\n")
endif()

configure_file(quadrotor_plugins.urdf.xacro.in ${PROJECT_SOURCE_DIR}/urdf/quadrotor_plugins.urdf.xacro @ONLY)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/urdf/quadrotor_plugins.urdf.xacro)

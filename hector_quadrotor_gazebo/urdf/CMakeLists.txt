# This CMakeLists.txt configures the plugins used for the quadrotor model.
set(MODEL_PLUGINS)

# configuration
option(USE_EXTERNAL_CONTROLLER "Do not run the hector_quadrotor_controller_gazebo plugin." OFF)
option(USE_PROPULSION_PLUGIN "Use a model of the quadrotor propulsion system" ON)
option(USE_AERODYNAMICS_PLUGIN "Use a model of the quadrotor aerodynamics" ON)

# sensor plugins
list(APPEND MODEL_PLUGINS quadrotor_sensors)

# controller plugin
if(NOT USE_EXTERNAL_CONTROLLER)
  list(APPEND MODEL_PLUGINS quadrotor_controller)
endif()

# propulsion plugin
#if(USE_PROPULSION_PLUGIN)
#  find_library(hector_gazebo_quadrotor_propulsion_LIBRARY hector_gazebo_quadrotor_propulsion)

#  if(NOT TARGET hector_gazebo_quadrotor_propulsion AND NOT hector_gazebo_quadrotor_propulsion_LIBRARY)
#    message(SEND_ERROR "Cannot use the propulsion model as the required Gazebo plugin 'hector_gazebo_quadrotor_propulsion' has not been found.")
#    set(USE_PROPULSION_PLUGIN OFF)
#  endif()
#endif()
if(USE_PROPULSION_PLUGIN)
  list(APPEND MODEL_PLUGINS quadrotor_propulsion)
endif()

# aerodynamics plugin
#if(USE_AERODYNAMICS_PLUGIN)
#  find_library(hector_gazebo_quadrotor_aerodynamics_LIBRARY hector_gazebo_quadrotor_aerodynamics)

#  if(NOT TARGET hector_gazebo_quadrotor_aerodynamics AND NOT hector_gazebo_quadrotor_aerodynamics_LIBRARY)
#    message(SEND_ERROR "Cannot use the aerodynamics model as the required Gazebo plugin 'hector_gazebo_quadrotor_aerodynamics' has not been found.")
#    set(USE_AERODYNAMICS_PLUGIN OFF)
#  endif()
#endif()
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

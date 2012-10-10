if(USE_AERODYNAMICS_MODEL)
  set(CONTROLLER_URDF "quadrotor_aerodynamics.urdf.xacro")
else()
  set(CONTROLLER_URDF "quadrotor_simple_controller.urdf.xacro")
endif()

configure_file(quadrotor_plugins.urdf.xacro.in ${PROJECT_SOURCE_DIR}/urdf/quadrotor_plugins.urdf.xacro @ONLY)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/urdf/quadrotor_plugins.urdf.xacro)


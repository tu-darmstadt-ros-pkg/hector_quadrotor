if(USE_PROPULSION_MODEL)
  set(URDF "${URDF}  <quadrotor_propulsion/>\n")
else()
  set(URDF "${URDF}  <quadrotor_simple_controller/>\n")
endif()

if(USE_AERODYNAMICS_MODEL)
  set(URDF "${URDF}  <quadrotor_aerodynamics/>\n")
endif()

configure_file(quadrotor_plugins.urdf.xacro.in ${PROJECT_SOURCE_DIR}/urdf/quadrotor_plugins.urdf.xacro @ONLY)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/urdf/quadrotor_plugins.urdf.xacro)


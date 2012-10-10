find_program(MATLAB NAMES matlab)

if (MATLAB)
  message(STATUS "Using Matlab at ${MATLAB}")
  file(GLOB MFILES "*.m")

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/emcprj/rtwlib/quadrotorPropulsion/quadrotorPropulsion.a ${CMAKE_CURRENT_SOURCE_DIR}/emcprj/rtwlib/quadrotorDrag/quadrotorDrag.a
    COMMAND ${MATLAB} -nodesktop -nosplash -r "compile_all; quit;"
    DEPENDS ${MFILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    VERBATIM
  )

  add_custom_target(quadrotor_model
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/emcprj/rtwlib/quadrotorPropulsion/quadrotorPropulsion.a ${CMAKE_CURRENT_SOURCE_DIR}/emcprj/rtwlib/quadrotorDrag/quadrotorDrag.a
    SOURCES ${MFILES}
  )

else(MATLAB)
  message(WARNING "Matlab not found")
endif(MATLAB)

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/emcprj)

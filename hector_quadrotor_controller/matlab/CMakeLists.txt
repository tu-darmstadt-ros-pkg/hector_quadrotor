find_program(MATLAB NAMES matlab)

if (MATLAB)
  message(STATUS "Using Matlab at ${MATLAB}")
  file(GLOB MFILES "*.m")

  include_directories(codegen/lib)

  add_custom_command(
    OUTPUT quadrotorPropulsion.a quadrotorDrag.a
    COMMAND ${MATLAB} -nojvm -nodesktop -nosplash -r "compile_all; quit;"
    COMMAND ${CMAKE_COMMAND} -E copy codegen/lib/quadrotorPropulsion/quadrotorPropulsion.a ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy codegen/lib/quadrotorDrag/quadrotorDrag.a ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${MFILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    VERBATIM
  )

  add_custom_target(quadrotor_model
    DEPENDS quadrotorPropulsion.a quadrotorDrag.a
  )

else(MATLAB)
  message(WARNING "Matlab not found")
endif(MATLAB)

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/codegen)

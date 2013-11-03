find_program(MATLAB NAMES matlab)

if(MATLAB)
  message(STATUS "Using Matlab at ${MATLAB}")
  file(GLOB MFILES "*.m")

  include_directories(codegen/lib)

  add_custom_command(
    OUTPUT codegen/lib/quadrotorPropulsion/quadrotorPropulsion.a codegen/lib/quadrotorDrag/quadrotorDrag.a
    COMMAND ${MATLAB} -nojvm -nodesktop -nosplash -r "compile_all; quit;"
    DEPENDS ${MFILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    VERBATIM
  )

#  add_library(quadrotorPropulsion STATIC IMPORTED GLOBAL)
#  set_target_properties(quadrotorPropulsion PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/codegen/lib/quadrotorPropulsion/quadrotorPropulsion.a)

  # workaround for cmake <2.8.8
  add_library(quadrotorPropulsion STATIC dummy.c)
  target_link_libraries(quadrotorPropulsion ${CMAKE_CURRENT_SOURCE_DIR}/codegen/lib/quadrotorPropulsion/quadrotorPropulsion.a)

#  add_library(quadrotorDrag STATIC IMPORTED GLOBAL)
#  set_target_properties(quadrotorDrag PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/codegen/lib/quadrotorDrag/quadrotorDrag.a)

  # workaround for cmake <2.8.8
  add_library(quadrotorDrag STATIC dummy.c)
  target_link_libraries(quadrotorDrag ${CMAKE_CURRENT_SOURCE_DIR}/codegen/lib/quadrotorDrag/quadrotorDrag.a)

else()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/codegen/lib/quadrotorPropulsion/quadrotorPropulsion.a)
    message(STATUS "Found precompiled hector_quadrotor propulsion model.")

#    add_library(quadrotorPropulsion STATIC IMPORTED GLOBAL)
#    set_target_properties(quadrotorPropulsion PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/codegen/lib/quadrotorPropulsion/quadrotorPropulsion.a)

    # workaround for cmake <2.8.8
    add_library(quadrotorPropulsion STATIC dummy.c)
    target_link_libraries(quadrotorPropulsion ${CMAKE_CURRENT_SOURCE_DIR}/codegen/lib/quadrotorPropulsion/quadrotorPropulsion.a)

  endif()

  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/codegen/lib/quadrotorDrag/quadrotorDrag.a)
    message(STATUS "Found precompiled hector_quadrotor drag model.")

#    add_library(quadrotorDrag STATIC IMPORTED GLOBAL)
#    set_target_properties(quadrotorDrag PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/codegen/lib/quadrotorDrag/quadrotorDrag.a)

    # workaround for cmake <2.8.8
    add_library(quadrotorDrag STATIC dummy.c)
    target_link_libraries(quadrotorDrag ${CMAKE_CURRENT_SOURCE_DIR}/codegen/lib/quadrotorDrag/quadrotorDrag.a)
  endif()

  if(NOT TARGET quadrotorPropulsion OR NOT TARGET quadrotorDrag)
    message(WARNING "Matlab not found. Cannot build hector_quadrotor_model libraries.")
  endif()
endif(MATLAB)

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/codegen)

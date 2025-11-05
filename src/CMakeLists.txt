
add_subdirectory(robot_interface)
add_subdirectory(seam_tracker)
add_subdirectory(ui)
add_subdirectory(robot_webservice)

add_subdirectory(interpolation)

install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake
  NAMESPACE ${PROJECT_NAME}::
)

# Generate basic config file of cmake package
file(GENERATE OUTPUT ${CMAKE_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake
  CONTENT "include(\"\${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Targets.cmake\")"
)

install(FILES "${CMAKE_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake
)

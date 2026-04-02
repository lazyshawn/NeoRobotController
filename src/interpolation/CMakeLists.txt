
add_library(interpolation
  InterpCurve.cpp
  InterpDispatch.cpp
  InterpSegment.cpp
  InterpSegmentBuffer.cpp
)
#set_target_properties(zmotion_interface PROPERTIES
#  LINK_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib/zmotion;${CMAKE_SOURCE_DIR}/lib/clogger"
#)
target_link_libraries(interpolation
  common
)
target_include_directories(interpolation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
#get_target_property(LINK_DIRS FSAIRobotInterface LINK_DIRECTORIES)
#message("Link directories: ${LINK_DIRS}")

# Then, link libraries

# Install headers
install(DIRECTORY include/interpolation
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

#install(TARGETS interpolation EXPORT ${PROJECT_NAME}Targets
#  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
#)

add_subdirectory(test)



add_library(common
  AuxMatrix.c
  AuxRotRep.c
  AuxTransformation.c
  GeoKinematics.c
  GeoFk.c
  GeoIK.c
  GeoIK_subproblem.c

  TrajectorySegment.cpp
)

target_include_directories(common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
#get_target_property(LINK_DIRS FSAIRobotInterface LINK_DIRECTORIES)
#message("Link directories: ${LINK_DIRS}")

# Then, link libraries

# Install headers
install(DIRECTORY include/common
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

#install(TARGETS common EXPORT ${PROJECT_NAME}Targets
#  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
#)


if(BUILD_TESTING)
  add_subdirectory(test)
endif()
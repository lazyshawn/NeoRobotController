
add_library(task_plan
  collision.cpp
)

target_include_directories(task_plan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
#get_target_property(LINK_DIRS FSAIRobotInterface LINK_DIRECTORIES)
#message("Link directories: ${LINK_DIRS}")

# Then, link libraries
find_package(ccd PATHS "${CMAKE_SOURCE_DIR}/extern/libccd/lib/ccd" REQUIRED)
find_package(fcl PATHS "${CMAKE_SOURCE_DIR}/extern/fcl/CMake" REQUIRED)
target_link_libraries(task_plan PUBLIC
  ccd
  fcl
)

# Install headers
install(DIRECTORY include/task_plan
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

#install(TARGETS task_plan EXPORT ${PROJECT_NAME}Targets
#  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
#)

add_subdirectory(test)

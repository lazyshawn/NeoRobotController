find_package(Eigen3 REQUIRED)

add_library(FSAIRobotInterface
  CoopRobotBase.cpp
  CoopRobotManager.cpp
  RobotTrajectory.cpp
  #BufferSynchronizer.cpp

  NeoRobot.cpp
)

target_include_directories(FSAIRobotInterface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/clogger>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/zmotion>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/openssl-3/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_directories(FSAIRobotInterface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/zmotion>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/clogger>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/openssl-3>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
)
#get_target_property(LINK_DIRS FSAIRobotInterface LINK_DIRECTORIES)
#message("Link directories: ${LINK_DIRS}")

# Then, link libraries
target_link_libraries(FSAIRobotInterface PUBLIC
  #zauxdll
  #zmotion
  log4cplus$<$<AND:$<CONFIG:Debug>,$<PLATFORM_ID:Windows>>:D>
  Eigen3::Eigen

  #common
  interpolation
)

# Install headers
install(DIRECTORY include/robot_interface
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(TARGETS FSAIRobotInterface EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_subdirectory(test)

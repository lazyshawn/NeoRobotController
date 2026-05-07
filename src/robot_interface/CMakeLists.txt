
add_library(FSAIRobotInterface
  ZMotionController.cpp
  CoopRobot.cpp
  RobotTrajectory.cpp
  ParamSerialization.cpp
  BufferSynchronizer.cpp

  FSAIRobot.cpp
  FSAISimRobot.cpp
  ZMotionRobot.cpp
  ZRVRobot.cpp
)
#set_target_properties(zmotion_interface PROPERTIES
#  LINK_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib/zmotion;${CMAKE_SOURCE_DIR}/lib/clogger"
#)
#target_link_libraries(FSAIRobotInterface debug ${Debug_Path}/LibName release ${Release_Path}/LibName)
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
  zauxdll
  zmotion
  log4cplus$<$<CONFIG:Debug>:D>
  libssl
  libcrypto
  fsai_simulator/QtWidgetsApplication3
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

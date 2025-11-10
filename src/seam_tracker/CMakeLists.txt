

add_library(zmotion_tracker
  zmotion_tracker_interface.cpp
)
target_include_directories(zmotion_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/clogger>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/zmotion>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_subdirectory(test)

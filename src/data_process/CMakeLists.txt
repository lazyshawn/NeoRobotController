

add_library(data_process
  arc_tracker.c
  control_algo.c
  fir_filter.c
  track_utility.c

  input_shaping.c
)
target_include_directories(data_process PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(data_process PUBLIC
  common
)

add_subdirectory(test)

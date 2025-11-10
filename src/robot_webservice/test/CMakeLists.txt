
add_executable(demo_webservice
  demo_webservice.cpp
)
target_include_directories(demo_webservice PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(demo_webservice PUBLIC
  robot_webservice
)
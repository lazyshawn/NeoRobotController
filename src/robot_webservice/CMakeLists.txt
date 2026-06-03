

add_library(robot_webservice
  webservice.cpp
)
target_include_directories(robot_webservice PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/openssl-3/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_directories(robot_webservice PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/openssl-3>
)

target_link_libraries(robot_webservice PUBLIC
  libssl
  libcrypto
  common
)

# Install headers
install(DIRECTORY include/robot_webservice
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(TARGETS robot_webservice EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_subdirectory(test)

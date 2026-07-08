# OBJECT 库，只编译，不链接，包含全部源文件
add_library(interpolation_obj OBJECT
  InterpCurve.cpp
  InterpSegment.cpp
  InterpSegmentBuffer.cpp
  InterpDispatch.cpp
)
target_link_libraries(interpolation_obj PUBLIC
  common
)
target_include_directories(interpolation_obj PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
#get_target_property(LINK_DIRS FSAIRobotInterface LINK_DIRECTORIES)
#message("Link directories: ${LINK_DIRS}")

# 共享库：依赖 OBJECT 库，usage requirements 自动继承（include 路径 + common 链接）
add_library(interpolation)
target_link_libraries(interpolation PUBLIC interpolation_obj)

# Install headers
install(DIRECTORY include/interpolation
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# 两者同属一个 export set，OBJECT 库不会产生二进制安装产物，仅为满足依赖链
install(TARGETS interpolation interpolation_obj EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_subdirectory(test)

cmake_minimum_required(VERSION 3.15)
project(RobotInterface VERSION 1.0.0)

if(WIN32)
  # create symbol for exporting lib for dynmic library
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  # control where the static and shared libraries are built so that on windows
  # we don't need to tinker with the path to run the executable
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  # Remove 'lib' prefix for shared libraries on Windows
  set(CMAKE_SHARED_LIBRARY_PREFIX "")
  # Build static in windows
  #option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
  set(BUILD_SHARED_LIBS ON CACHE BOOL "Build using shared libraries"  FORCE)
  # Set the runtime lib for MSCV (global)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
else()
  option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
endif()

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/output/${PROJECT_NAME})
include(GNUInstallDirs)


# 添加一个选项，默认开启测试
option(BUILD_TESTING "Build and run tests" ON)
if(BUILD_TESTING)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
  )
  if(WIN32)
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  set_target_properties(gtest PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  set_target_properties(gtest_main PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

  # 启用测试，这是 CTest 所必需的
  enable_testing()

  # 添加包含测试的子目录，比如将测试代码放在 test/ 文件夹下
  #add_subdirectory(tests)
endif()


add_subdirectory(extern)
add_subdirectory(src)
add_subdirectory(test)

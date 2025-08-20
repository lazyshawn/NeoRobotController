cmake_minimum_required(VERSION 3.15)
project(ZauxRobot VERSION 1.0.0)

if(WIN32)
  # create symbol for exporting lib for dynmic library
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  # control where the static and shared libraries are built so that on windows
  # we don't need to tinker with the path to run the executable
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  # Remove 'lib' prefix for shared libraries on Windows
  # set(CMAKE_SHARED_LIBRARY_PREFIX "")
  # Build static in windows
  option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)
  # Set the runtime lib for MSCV (global)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
else()
  option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
endif()

include(GNUInstallDirs)

add_subdirectory(extern)
add_subdirectory(src)
add_subdirectory(test)

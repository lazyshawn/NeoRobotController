set(QT5_DIR "D:/Qt/Qt5.12.8/5.12.8/msvc2017_64")
find_package(Qt5 PATHS "${QT5_DIR}/lib/cmake/Qt5" REQUIRED COMPONENTS Core Widgets LinguistTools Concurrent)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/forms)

# Create executable for MVVM demo
add_executable(demo_mvvm
  main.cpp
  # models
  MainModel.cpp
  # viewmodels
  MainViewModel.cpp
  # views
  mainView.cpp
)

target_link_libraries(demo_mvvm PRIVATE
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::Concurrent
  FSAIRobotInterface
)

target_include_directories(demo_mvvm PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/mvvm_demo_autogen/include$<$<BOOL:${CMAKE_CONFIGURATION_TYPES}>:_$<CONFIG>>>"
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if (EXISTS "${QT5_DIR}/bin/windeployqt.exe")
    message("windeployqt exists")
    add_custom_command(TARGET demo_mvvm POST_BUILD
        COMMAND "${QT5_DIR}/bin/windeployqt.exe" "--compiler-runtime"
        "${PROJECT_BINARY_DIR}$<$<BOOL:${CMAKE_CONFIGURATION_TYPES}>:/$<CONFIG>>/demo_mvvm.exe"
    )
endif()

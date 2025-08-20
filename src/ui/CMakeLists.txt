
set(QT5_DIR "D:/Qt/Qt5.12.8/5.12.8/msvc2017_64")
find_package(Qt5 PATHS "${QT5_DIR}/lib/cmake/Qt5" REQUIRED COMPONENTS Core Widgets LinguistTools Concurrent)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/forms)

# Ui
add_library(qt_ui
  fsai_mainWindow.cpp
  fsai_procedure.cpp
  fsai_advanceConfigWindow.cpp
)
target_link_libraries(qt_ui PUBLIC
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::Concurrent
)
target_include_directories(qt_ui PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/qt_ui_autogen/include$<$<BOOL:${CMAKE_CONFIGURATION_TYPES}>:_$<CONFIG>>>"
  #$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/qt_ui_autogen/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# FSAI app: ÐÅºÅºÍ²Û
add_library(fsai_app
  fsai_app.cpp
)
target_link_libraries(fsai_app PUBLIC
  qt_ui
  FSAIRobotInterface
)
target_include_directories(fsai_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_subdirectory(test)
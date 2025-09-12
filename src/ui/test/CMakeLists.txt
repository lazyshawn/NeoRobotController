
add_executable(test_fsai_app# WIN32
  test.cpp
  ../forms/logo.rc
)
target_link_libraries(test_fsai_app PUBLIC
  fsai_app
)
set_target_properties(test_fsai_app
  PROPERTIES WIN32_EXECUTABLE $<IF:$<CONFIG:Release>,ON,OFF>
)

##[[
if (EXISTS "${QT5_DIR}/bin/windeployqt.exe")
    message("windeployqt exists")
    add_custom_command(TARGET test_fsai_app POST_BUILD
        COMMAND "${QT5_DIR}/bin/windeployqt.exe" "--compiler-runtime"
        "${PROJECT_BINARY_DIR}$<$<BOOL:${CMAKE_CONFIGURATION_TYPES}>:/$<CONFIG>>/test_fsai_app.exe"
    )
endif()
#]]
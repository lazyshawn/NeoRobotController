
### FSAIRobot
add_executable(demoInterpSegment
  demoInterpSegment.cpp
)
target_include_directories(demoInterpSegment
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_link_libraries(demoInterpSegment PUBLIC
  interpolation_obj
)
# 取消 DLL 导入/导出语义
#target_compile_definitions(demoInterpSegment PRIVATE EXPORT_STATIC_LIBS)
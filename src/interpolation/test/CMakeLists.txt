
### FSAIRobot
add_executable(demoInterpSegment
  demoInterpSegment.cpp
)
target_link_libraries(demoInterpSegment PUBLIC
  interpolation
)
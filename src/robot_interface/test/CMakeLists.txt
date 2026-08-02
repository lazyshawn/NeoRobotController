### NeoRobot
add_executable(demo_NeoRobot
  demo_NeoRobot.cpp
)
target_link_libraries(demo_NeoRobot PUBLIC
  FSAIRobotInterface
)
# DbgHelp 仅 Windows 存在
if(WIN32)
  target_link_libraries(demo_NeoRobot PUBLIC DbgHelp)
endif()

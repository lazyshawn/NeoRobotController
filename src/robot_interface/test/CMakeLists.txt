
### FSAIRobot
add_executable(demoFSAIRobot
  demoFSAIRobot.cpp
)
target_link_libraries(demoFSAIRobot PUBLIC
  DbgHelp
)

### ZMotionRobot
add_executable(demoZMotionRobot
  demoZMotionRobot.cpp
)
target_link_libraries(demoZMotionRobot PUBLIC
  FSAIRobotInterface
)

### NeoRobot
add_executable(demo_NeoRobot
  demo_NeoRobot.cpp
)
target_link_libraries(demo_NeoRobot PUBLIC
  FSAIRobotInterface
  DbgHelp
)

### RobotTrajectory
add_executable(test_trajectory
  test_trajectory.cpp
)
target_link_libraries(test_trajectory PUBLIC
  FSAIRobotInterface
)

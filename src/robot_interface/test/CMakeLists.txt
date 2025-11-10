
### FSAIRobot
add_executable(demoFSAIRobot
  demoFSAIRobot.cpp
)
target_link_libraries(demoFSAIRobot PUBLIC
  FSAIRobotInterface
)

### ZMotionRobot
add_executable(demoZMotionRobot
  demoZMotionRobot.cpp
)
target_link_libraries(demoZMotionRobot PUBLIC
  FSAIRobotInterface
)

### ZRVRobot
add_executable(demoZRVRobot
  demoZRVRobot.cpp
)
target_link_libraries(demoZRVRobot PUBLIC
  FSAIRobotInterface
)

### RobotTrajectory
add_executable(test_trajectory
  test_trajectory.cpp
)
target_link_libraries(test_trajectory PUBLIC
  FSAIRobotInterface
)


add_executable(demo_tracker
  demo_tracker.cpp
)
# Then, link libraries
target_link_libraries(demo_tracker PUBLIC
  zmotion_tracker
)

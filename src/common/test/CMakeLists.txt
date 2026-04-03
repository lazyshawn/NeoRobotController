### test_common
add_executable(test_common
  test_common.cpp
)
target_link_libraries(test_common PUBLIC
  common
  GTest::GTest
)
include(GoogleTest)
gtest_discover_tests(test_common)

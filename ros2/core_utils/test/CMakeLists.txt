find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(fmt_test
  test_fmt_format.cpp
)

target_link_libraries(fmt_test core_utils)

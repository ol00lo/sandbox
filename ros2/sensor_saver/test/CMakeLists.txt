find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)

ament_add_gtest(test_ttl
  test_ttl.cpp
)
if(TARGET test_ttl)
  target_link_libraries(test_ttl sensor_saver_impl rclcpp::rclcpp)
endif()



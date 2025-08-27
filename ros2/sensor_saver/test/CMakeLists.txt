find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)

ament_add_gtest(mouse_sensor_test
  main.cpp
  mouse_sensor_test.cpp
)
target_link_libraries(mouse_sensor_test sensor_saver_impl)

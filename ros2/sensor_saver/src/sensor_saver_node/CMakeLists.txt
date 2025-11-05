ament_auto_add_executable(sensor_saver_node
  sensor_saver_node.cpp
)

target_link_libraries(sensor_saver_node
  sensor_saver_impl
  rclcpp::rclcpp
)

install(TARGETS sensor_saver_node
  DESTINATION lib/${PROJECT_NAME})

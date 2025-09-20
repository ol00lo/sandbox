find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(sensor_saver_node src/sensor_saver_node.cpp)
target_link_libraries(sensor_saver_node PRIVATE
  sensor_saver_impl
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

install(TARGETS sensor_saver_node
  DESTINATION lib/${PROJECT_NAME})
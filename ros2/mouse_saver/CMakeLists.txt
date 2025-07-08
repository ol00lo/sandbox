cmake_minimum_required(VERSION 3.8)
project(mouse_saver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(PQXX_INCLUDE_DIR "/usr/include")
set(PQXX_LIBRARY "/usr/lib/x86_64-linux-gnu/libpqxx.so")

add_executable(mouse_saver mouse_control/mouse_saver.cpp)

target_include_directories(mouse_saver PRIVATE ${PQXX_INCLUDE_DIR})
target_link_libraries(mouse_saver PRIVATE
  ${PQXX_LIBRARY}
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS mouse_saver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
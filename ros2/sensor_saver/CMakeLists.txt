cmake_minimum_required(VERSION 3.8)
project(sensor_saver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(PQXX_INCLUDE_DIR "/usr/include")
set(PQXX_LIBRARY "/usr/lib/x86_64-linux-gnu/libpqxx.so")

add_executable(sensor_saver src/sensor_saver_node.cpp)

target_include_directories(sensor_saver PRIVATE ${PQXX_INCLUDE_DIR})
target_link_libraries(sensor_saver PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${PQXX_LIBRARY}
)
target_compile_options(sensor_saver PRIVATE
    $<$<CONFIG:Release>:-Werror>
    $<$<CONFIG:Debug>:-Wno-error>
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wunused-const-variable
    -Wduplicated-cond
    -Wnon-virtual-dtor
    -Woverloaded-virtual
    -Wredundant-decls
    -Wsign-promo
    -Wzero-as-null-pointer-constant
    -Wsuggest-override
    -Wshadow
    -Wfloat-equal
    -Wcast-qual
    -Wlogical-op
    -Wcast-align
    -Wold-style-cast
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS sensor_saver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
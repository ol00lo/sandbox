cmake_minimum_required(VERSION 3.8)
project(sensor_saver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(core_utils REQUIRED)

add_subdirectory(src)

# if(BUILD_TESTING)
#   add_subdirectory(test)
# endif()

# ament_auto_package()


ament_export_include_directories(include)
ament_export_libraries(sensor_saver_impl)
ament_export_dependencies(rclcpp std_msgs)
ament_package()
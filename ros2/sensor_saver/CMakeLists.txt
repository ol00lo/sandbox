cmake_minimum_required(VERSION 3.8)
project(sensor_saver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(src)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_auto_package()
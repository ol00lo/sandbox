find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)

add_library(core_utils INTERFACE)

target_include_directories(core_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(core_utils INTERFACE fmt::fmt)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/core_utils.hpp
  DESTINATION include/core_utils
)

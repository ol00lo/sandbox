find_package(rclcpp REQUIRED)

add_library(core_utils core_utils.cpp)
target_include_directories(core_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

find_package(fmt REQUIRED)
target_link_libraries(core_utils fmt::fmt)

install(TARGETS core_utils
  DESTINATION lib/${PROJECT_NAME}
)

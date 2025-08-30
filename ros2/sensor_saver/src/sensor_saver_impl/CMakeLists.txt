set(PQXX_INCLUDE_DIR "/usr/include")
set(PQXX_LIBRARY "/usr/lib/x86_64-linux-gnu/libpqxx.so")


add_library(sensor_saver_impl sensor_saver_impl.cpp)
target_include_directories(sensor_saver_impl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(sensor_saver_impl PRIVATE ${PQXX_INCLUDE_DIR})
target_link_libraries(sensor_saver_impl PUBLIC ${PQXX_LIBRARY} rclcpp::rclcpp)


install(TARGETS sensor_saver_impl
  DESTINATION lib/${PROJECT_NAME}
)

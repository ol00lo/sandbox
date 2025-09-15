set(PQXX_INCLUDE_DIR "/usr/include")
set(PQXX_LIBRARY "/usr/lib/x86_64-linux-gnu/libpqxx.so")

add_library(sensor_saver_impl sensor_saver_impl.cpp)

target_include_directories(sensor_saver_impl
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${PQXX_INCLUDE_DIR} core_utils::core_utils
)

target_link_libraries(sensor_saver_impl 
    PUBLIC ${PQXX_LIBRARY} rclcpp::rclcpp core_utils::core_utils
)

install(TARGETS sensor_saver_impl
  DESTINATION lib/${PROJECT_NAME}
)
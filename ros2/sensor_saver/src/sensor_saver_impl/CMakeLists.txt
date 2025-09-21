# Сначала находим ВСЕ зависимости
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(core_utils REQUIRED)  # ДОЛЖНО БЫТЬ ПЕРЕД использованием


set(PQXX_INCLUDE_DIR "/usr/include")
set(PQXX_LIBRARY "/usr/lib/x86_64-linux-gnu/libpqxx.so")


add_library(sensor_saver_impl sensor_saver_impl.cpp)

# СОВРЕМЕННЫЙ способ указания зависимостей (вместо ament_target_dependencies)
target_link_libraries(sensor_saver_impl PUBLIC
    rclcpp::rclcpp
    core_utils
    ${PQXX_LIBRARY}
)

# Добавляем include директории
target_include_directories(sensor_saver_impl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/
    ${PQXX_INCLUDE_DIR}
)

# Для экспорта зависимостей (если это библиотека)
ament_export_dependencies(rclcpp core_utils)
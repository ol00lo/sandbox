set(PQXX_INCLUDE_DIR "/usr/include")
set(PQXX_LIBRARY "/usr/lib/x86_64-linux-gnu/libpqxx.so")

ament_auto_add_library(sensor_saver_impl SHARED
  sensor_saver_impl.cpp
)

target_include_directories(sensor_saver_impl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
  ${PQXX_INCLUDE_DIR}
)

target_link_libraries(sensor_saver_impl
  ${PQXX_LIBRARY}
)

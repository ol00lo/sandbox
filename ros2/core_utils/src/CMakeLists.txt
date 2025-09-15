add_library(core_utils SHARED core_utils.cpp)

target_include_directories(core_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(core_utils PUBLIC fmt::fmt)

install(TARGETS core_utils
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(FILES
  fmt_format.hpp
  DESTINATION include
)
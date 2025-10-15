add_library(core_utils SHARED core_utils/core_utils.cpp)

target_include_directories(core_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(core_utils PUBLIC fmt::fmt)
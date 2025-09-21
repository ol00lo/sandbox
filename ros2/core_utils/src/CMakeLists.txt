add_library(core_utils SHARED core_utils/core_utils.cpp)
add_library(core_utils::core_utils ALIAS core_utils)

target_include_directories(core_utils
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)

target_link_libraries(core_utils PUBLIC fmt::fmt)
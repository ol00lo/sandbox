cmake_minimum_required(VERSION 3.8)
project(core_utils)

find_package(ament_cmake_auto REQUIRED)
find_package(fmt REQUIRED)

ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(src)
add_subdirectory(test)

add_compile_options(
  $<$<CONFIG:Release>:-Werror>
  $<$<CONFIG:Debug>:-Wno-error>
  $<$<CONFIG:RelWithDebInfo>:-Werror>
  -Wall
  -Wextra
  -Wpedantic
  -Wconversion
  -Wunused-const-variable
  -Wduplicated-cond
  -Wnon-virtual-dtor
  -Woverloaded-virtual
  -Wredundant-decls
  -Wsign-promo
  -Wzero-as-null-pointer-constant
  -Wsuggest-override
  -Wshadow
  -Wfloat-equal
  -Wcast-qual
  -Wlogical-op
  -Wcast-align
  -Wold-style-cast
  -Wmissing-field-initializers
)

ament_auto_package()

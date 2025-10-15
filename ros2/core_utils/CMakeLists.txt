cmake_minimum_required(VERSION 3.8)
project(core_utils)

# Используем ament_auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()  # Автоматически найдет зависимости из package.xml

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

add_subdirectory(src)

# ДОПОЛНИТЕЛЬНЫЕ ЭКСПОРТЫ ДО ament_auto_package()
ament_export_dependencies(fmt)

# АВТОМАТИЧЕСКАЯ НАСТРОЙКА (ВКЛЮЧАЕТ ament_package())
ament_auto_package()
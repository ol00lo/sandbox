#library

add_library(simlife_lib SHARED
	board.cpp
	cmd_viewer.cpp
	life_engine.cpp
	driver.cpp
	simlife_c_api.cpp
)

target_compile_definitions(simlife_lib PRIVATE LIFELIB_EXP)

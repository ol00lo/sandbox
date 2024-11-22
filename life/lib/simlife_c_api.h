#ifndef SIMLIFE_C_API_H
#define SIMLIFE_C_API_H
#include"simlife_lib.h"

extern "C" {
    LIFELIB_API int driver_create(char engine_type, char viewer_type, int width, int height, void** driver);
    LIFELIB_API int driver_set_delay(void* driver, int delay);
    LIFELIB_API int driver_set_init_conditions(void* driver, int* input, int input_size);
    LIFELIB_API int driver_start(void* driver, int* n_total_steps);
    LIFELIB_API int driver_destroy(void* driver);
    LIFELIB_API int driver_set_board_type(void* driver, char board_type);
}

#endif // SIMLIFE_C_API_H
#ifndef SIMLIFE_C_API_H
#define SIMLIFE_C_API_H

#ifdef _WIN32
    #ifdef simlife_EXPORTS
        #define LIFELIB_API __declspec(dllexport)
    #else
        #define LIFELIB_API __declspec(dllimport)
    #endif
#else
    #define LIFELIB_API
#endif

extern "C" {
    LIFELIB_API int driver_create(char engine_type, char viewer_type, int width, int height, void** driver);
    LIFELIB_API int driver_set_delay(void* driver, int delay);
    LIFELIB_API int driver_set_init_conditions(void* driver, int* input, int input_size);
    LIFELIB_API int driver_start(void* driver);
    LIFELIB_API int driver_destroy(void* driver);
    LIFELIB_API int driver_set_board_type(void* driver, char board_type);
}

#endif // SIMLIFE_C_API_H
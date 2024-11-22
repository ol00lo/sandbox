#include "simlife_c_api.h"
#include "driver.hpp"
#include <iostream>
#include <vector>

#define C_API_BEGIN                                                                                                    \
    try                                                                                                                \
    {
#define C_API_END                                                                                                      \
    }                                                                                                                  \
    catch (const std::exception& e)                                                                                    \
    {                                                                                                                  \
        std::cout << e.what() << std::endl;                                                                            \
        return 1;                                                                                                      \
    }

extern "C"
{
    int driver_create(char engine_type, char viewer_type, int width, int height, void** driver)
    {
        C_API_BEGIN
        *driver = new Driver(engine_type, viewer_type, width, height);
        return 0;
        C_API_END
    }

    int driver_set_delay(void* driver, int delay)
    {
        C_API_BEGIN
        Driver* d = static_cast<Driver*>(driver);
        d->set_delay(delay);
        return 0;
        C_API_END
    }

    int driver_set_init_conditions(void* driver, int* input, int input_size)
    {
        C_API_BEGIN
        Driver* d = static_cast<Driver*>(driver);
        std::vector<bool> init_conditions(input, input + input_size);
        d->set_init_conditions(init_conditions);
        return 0;
        C_API_END
    }

    int driver_start(void* driver)
    {
        C_API_BEGIN
        Driver* d = static_cast<Driver*>(driver);
        return d->start();
        C_API_END
    }

    int driver_destroy(void* driver)
    {
        C_API_BEGIN
        delete static_cast<Driver*>(driver);
        return 0;
        C_API_END
    }

    int driver_set_board_type(void* driver, char board_type)
    {
        C_API_BEGIN
        Driver* d = static_cast<Driver*>(driver);
        d->set_board_type(board_type);
        C_API_END
    }
}

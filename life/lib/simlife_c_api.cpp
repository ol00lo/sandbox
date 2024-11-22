#include "simlife_c_api.h"
#include "driver.hpp"
#include <iostream>
#include <vector>

#define C_API_BEGIN                                                                                                    \
    try                                                                                                                \
    {
#define C_API_END                                                                                                      \
    return 0;                                                                                                          \
    }                                                                                                                  \
    catch (const std::exception& e)                                                                                    \
    {                                                                                                                  \
        std::cout << e.what() << std::endl;                                                                            \
        return 1;                                                                                                      \
    }

int driver_create(char engine_type, char viewer_type, int width, int height, void** driver)
{
    C_API_BEGIN
    *driver = new Driver(engine_type, viewer_type, width, height);
    C_API_END
}

int driver_set_delay(void* driver, int delay)
{
    C_API_BEGIN
    Driver* d = static_cast<Driver*>(driver);
    d->set_delay(delay);
    C_API_END
}

int driver_set_init_conditions(void* driver, int* input, int input_size)
{
    C_API_BEGIN
    Driver* d = static_cast<Driver*>(driver);
    std::vector<bool> init_conditions(input, input + input_size);
    d->set_init_conditions(init_conditions);
    C_API_END
}

int driver_start(void* driver, int* n_total_steps)
{
    C_API_BEGIN
    Driver* d = static_cast<Driver*>(driver);
    *n_total_steps = d->start();
    C_API_END
}

int driver_destroy(void* driver)
{
    C_API_BEGIN
    delete static_cast<Driver*>(driver);
    C_API_END
}

int driver_set_board_type(void* driver, char board_type)
{
    C_API_BEGIN
    Driver* d = static_cast<Driver*>(driver);
    d->set_board_type(board_type);
    C_API_END
}

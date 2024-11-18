#include "arguments.hpp"]
#include "driver.hpp"
#include <iostream>

void run_new(int argc, char* argv[])
{
    Arguments argum(argc, argv);
    Driver d(argum.engine_type, argum.viewer_type, argum.width, argum.height);
    d.set_delay(argum.delay);
    d.set_init_conditions(argum.input);
    d.start();
}

int main(int argc, char* argv[])
{
    try
    {
        run_new(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}
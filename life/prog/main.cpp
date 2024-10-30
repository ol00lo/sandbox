#include "cxxopts.hpp"
#include "game.hpp"
#include "igame.h"
#include "simlife_utils.hpp"
#include <chrono>
#include <iostream>
#include <random>
#include <sstream>
#include <string>

void run_new(int argc, char* argv[])
{
    Arguments argum(argc, argv);
    Driver d(argum);
    d.start();
}
void run_new_new(int argc, char* argv[])
{
    Arguments argum(argc, argv);

    Driver* driver = driver_create(&argum);

    driver_start(driver);

    driver_destroy(driver);
}
int main(int argc, char* argv[])
{
    try
    {
        run_new_new(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}
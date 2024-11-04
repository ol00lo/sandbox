#include "cxxopts.hpp"
#include "driver.hpp"
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
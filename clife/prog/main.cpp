#include "arguments.hpp"
#include "board.h"
#include "cxxopts.hpp"
#include "game.h"
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <thread>

void test_game(int argc, char* argv[])
{
    Driver driver;
    Arguments arg(argc, argv);
    initialize_game(&driver, arg);
    run_game(driver);
}

int main(int argc, char* argv[])
{
    try
    {
        test_game(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}
#include "cxxopts.hpp"
#include "game.hpp"
#include "simlife_utils.hpp"
#include <chrono>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <thread>

int main(int argc, char* argv[])
{
    try
    {
        Arguments argum(argc, argv);
        GameOfLife game(argum);
        game.display();
        while (game.step())
        {
            game.display();
            std::this_thread::sleep_for(std::chrono::milliseconds(argum.delay));
        }
        game.over();
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}
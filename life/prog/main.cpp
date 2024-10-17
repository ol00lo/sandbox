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
        cxxopts::Options options("SimLife", "Simulation of life game");
        options.add_options()("dim", "dimention (int HEIGHTxint WIDTH)",
                              cxxopts::value<std::string>()->default_value("10x10"))                    //
            ("initfile", "Initial from file", cxxopts::value<std::string>())                            //
            ("random", "Random init with norganisms alive", cxxopts::value<int>()->default_value("20")) //
            ("delay", "Delay (milliseconds)", cxxopts::value<int>()->default_value("1000"))             //
            ("h,help", "Print help")                                                                    //
            ("bc", "Type of board: default-wall, p-periodic, m-mirror",                                 //
             cxxopts::value<char>()->default_value("l"));                                               //
        auto result = options.parse(argc, argv);

        Arguments argum(options, result);
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
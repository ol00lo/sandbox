#include "cxxopts.hpp"
#include "game.hpp"
#include "simlife.hpp"

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
            ("h,help", "Print help");                                                                   //

        auto result = options.parse(argc, argv);
        if (result.count("help"))
        {
            std::cout << options.help() << std::endl;
            exit(0);
        }

        std::string dimension = result["dim"].as<std::string>();
        size_t xPos = dimension.find('x');
        int width = std::stoi(dimension.substr(0, xPos));
        int height = std::stoi(dimension.substr(xPos + 1));

        int norganisms = result["random"].as<int>();
        int delay = result["delay"].as<int>();

        std::vector<bool> input;
        if (result.count("initfile"))
        {
            std::string filename = result["initfile"].as<std::string>();
            auto dim = dim_fromfile(filename);
            height = dim.first;
            width = dim.second;
            input = input_fromfile(filename);
        }
        else
        {
            input = random_input(height, width, norganisms);
        }

        GameOfLife game(height, width);
        game.initialize(std::move(input));
        game.display();
        while (game.step())
        {
            game.display();
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        }
        game.over();
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}
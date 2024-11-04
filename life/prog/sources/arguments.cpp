#include "arguments.hpp"
#include "simlife_utils.hpp"

Arguments::Arguments(int argc, char* argv[])
{
    cxxopts::Options options("SimLife", "Simulation of life game");
    options.add_options()("dim", "dimention (int HEIGHTxint WIDTH)", cxxopts::value<std::string>())   //
        ("initfile", "Initial from file", cxxopts::value<std::string>())                              //
        ("random", "Random init with norganisms alive", cxxopts::value<int>(norganisms))              //
        ("delay", "Delay (milliseconds)", cxxopts::value<int>(delay))                                 //
        ("bc", "Type of board: default-wall, p-periodic, m-mirror", cxxopts::value<char>(type_board)) //
        ("h,help", "Print help");                                                                     //

    auto result = options.parse(argc, argv);
    if (result.count("help"))
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    if (result.count("dim"))
    {
        std::tie(height, width) = dim_fromline(result["dim"].as<std::string>());
    }
    if (result.count("initfile"))
    {
        std::string filename = result["initfile"].as<std::string>();
        std::tie(height, width) = dim_fromfile(filename);
        input = input_fromfile(filename);
    }
    else
    {
        input = random_input(height, width, norganisms);
    }
}

void Arguments::validate()
{
    if (input.size() != width * height)
    {
        throw std::runtime_error("incorrect input vector");
    }
    if (width < 1 || height < 1)
    {
        throw std::runtime_error("incorrect dimentions");
    }
}
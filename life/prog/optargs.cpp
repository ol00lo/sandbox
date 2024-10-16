#include "optargs.hpp"
#include "simlife_utils.hpp"

Arguments::Arguments(cxxopts::Options options, cxxopts::ParseResult result)
{
    if (result.count("help"))
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }
    std::tie(height, width) = dim_fromline(result["dim"].as<std::string>());
    norganisms = result["random"].as<int>();
    delay = result["delay"].as<int>();

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
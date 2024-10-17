#ifndef OPTSARGS_HPP
#define OPTSARGS_HPP

#include "cxxopts.hpp"
#include <iostream>
struct Arguments
{
    Arguments(cxxopts::Options options, cxxopts::ParseResult result);
    int width;
    int height;
    int norganisms;
    int delay;
    std::vector<bool> input;
    void validate();
    char type_of_board;
};

#endif // !OPTSARGS_HPP
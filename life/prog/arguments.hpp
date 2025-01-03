#ifndef OPTSARGS_HPP
#define OPTSARGS_HPP

#include "cxxopts.hpp"
#include <iostream>

struct Arguments
{
    Arguments(int argc, char* argv[]);
    int width = 10;
    int height = 10;
    int norganisms = 20;
    int delay = 1000;
    std::vector<bool> input;
    void validate();
    char type_board = 'w';
    char viewer_type = 'c';
    char engine_type = 'c';
};

#endif // !OPTSARGS_HPP
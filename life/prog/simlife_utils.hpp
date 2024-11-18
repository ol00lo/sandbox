#ifndef SIMLIFE_UTILS_HPP
#define SIMLIFE_UTILS_HPP
#include "simlife_lib.hpp"
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

std::vector<bool> LIFELIB_API random_input(int height, int width, int norganisms);
std::vector<bool> LIFELIB_API input_fromfile(std::string filename);
std::pair<int, int> LIFELIB_API dim_fromfile(std::string filename);
std::pair<int, int> LIFELIB_API dim_fromline(std::string dimensions);
#endif // !SIMLIFE_UTILS_HPP
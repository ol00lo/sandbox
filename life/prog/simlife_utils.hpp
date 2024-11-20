#ifndef SIMLIFE_UTILS_HPP
#define SIMLIFE_UTILS_HPP
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

std::vector<bool> random_input(int height, int width, int norganisms);
std::vector<bool> input_fromfile(std::string filename);
std::pair<int, int> dim_fromfile(std::string filename);
std::pair<int, int> dim_fromline(std::string dimensions);
#endif // !SIMLIFE_UTILS_HPP
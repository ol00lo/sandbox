#ifndef SIMLIFE_HPP
#define SIMLIFE_HPP
#include <chrono>
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>
#include <string>
#include <thread>

std::vector<bool> random_input(int height, int width, int norganisms);
std::vector<bool> input_fromfile(std::string filename);
std::pair<int, int> dim_fromfile(std::string filename);
#endif // !SIMLIFE_HPP
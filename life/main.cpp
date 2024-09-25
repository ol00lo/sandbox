//#include "display.hpp"
#include"game.hpp"
//#include <SFML/Graphics.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <random>
#include <thread>
auto seed = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 ran(seed);
// std::mt19937 ran(0);

int main(int argc, char* argv[])
{
    int width = std::stoi(argv[1]);
    int height = std::stoi(argv[2]);
    int norganisms = std::stoi(argv[3]);
    // width = 15;
    // height = 13;
    // norganisms = 30;
    Game game(height, width);
    // game.initialize({{1, 1, 0}, {1, 0, 0}, {0, 1, 0}, {1, 0, 1}, {0, 1, 0}});

    std::vector<std::vector<bool>> in(height, std::vector<bool>(width, false));
    std::uniform_int_distribution row(0, height - 1);
    std::uniform_int_distribution col(0, width - 1);
    for (int i = 0; i < norganisms; i++)
    {
        int irow = row(ran);
        int icol = col(ran);
        in[irow][icol] = true;
    }
    game.initialize(in);
    while (game.step())
    {
        game.display();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    game.over();
}
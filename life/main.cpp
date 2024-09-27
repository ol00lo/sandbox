// #include "display.hpp"
#include "game.hpp"
// #include <SFML/Graphics.hpp>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char* argv[])
{
    try
    {
        int width = std::stoi(argv[1]);
        int height = std::stoi(argv[2]);
        int norganisms = std::stoi(argv[3]);
        width = 15;
        height = 17;
        norganisms = 40;
        GameOfLife game(height, width);
        // game.initialize({1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0});
        game.random_initialize(norganisms);

        game.display();
        while (game.step())
        {
            game.display();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        game.over();
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}
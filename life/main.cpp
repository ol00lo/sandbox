#include "display.hpp"
// #include "game.hpp"
#include <SFML/Graphics.hpp>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char* argv[])
{
    try
    {
        int width = std::stoi(argv[1]);
        int heigth = std::stoi(argv[2]);
        int norganisms = std::stoi(argv[3]);
        width = 15;
        heigth = 30;
        norganisms = 100;
        Screen screen(width, heigth, norganisms);
        // Screen screen(5, 3, 7);
        screen.run();
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }

    sf::RenderWindow window;
    window.create(sf::VideoMode(1400, 700), "Bd");
}

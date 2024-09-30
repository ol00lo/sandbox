#include "display.hpp"
// #include "game.hpp"
#include <SFML/Graphics.hpp>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

int main()
{
    try
    {
        Screen screen(5, 10, 17);
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

/*
int main(int argc, char* argv[])
{
    try
    {
        int _width = std::stoi(argv[1]);
        int _height = std::stoi(argv[2]);
        int _norganisms = std::stoi(argv[3]);
        _width = 15;
        _height = 17;
        _norganisms = 40;
        GameOfLife game(_height, _width);
        // game.initialize({1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0});
        game.random_initialize(_norganisms);

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
}*/
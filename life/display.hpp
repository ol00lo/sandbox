#include "game.hpp"
#include <SFML/Graphics.hpp>

class Screen
{
public:
    Screen(int rows, int cols, int norganisms);
    void run();

private:
    void handleEvents(char& gameActive);
    void updateGame(GameOfLife& g, int rows, int cols);
    void initializeButtons();
    void drawStartScreen();
    void drawGameOver(bool what);
    void showGameOver(bool what);
    bool isButtonClicked(const sf::RectangleShape& Button, const sf::Event& event);

    sf::RenderWindow _window;
    sf::Font font;
    sf::RectangleShape randomButton;
    sf::RectangleShape inputButton;
    sf::RectangleShape exitButton;
    sf::RectangleShape stopButton;
    sf::Text randomButtonText;
    sf::Text inputButtonText;
    sf::Text exitButtonText;
    sf::Text stopButtonText;
    bool startButtonsActive = true;

    sf::Texture cellTexture;
    sf::Sprite cellSprite;

    const int _width = 1400;
    const int _height = 700;

    const int _ncols, _nrows, _norganisms;
    int cellsize = (_width / _ncols > _height / _nrows) ? _height / _nrows : _width / _ncols;
};

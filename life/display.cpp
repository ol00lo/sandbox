#include "display.hpp"

Screen::Screen(int rows, int cols, int norganisms) : _nrows(rows), _ncols(cols), _norganisms(norganisms)
{
    if (!font.loadFromFile("resources/Samson.ttf"))
    {
        std::cerr << "Error loading font or texture\n";
    }
    if (!cellTexture.loadFromFile("resources/cat.png"))
    {
        std::cerr << "Error loading texture" << std::endl;
    }
    cellSprite.setTexture(cellTexture);
    initializeButtons();
}

void Screen::run()
{
    _window.create(sf::VideoMode(_width, _height), "World");
    char gameActive = 0;
    while (_window.isOpen())
    {
        handleEvents(gameActive);
        _window.clear();
        if (gameActive == 1)
        {
            startButtonsActive = false;
            GameOfLife g(_nrows, _ncols);
            g.random_initialize(_norganisms);
            updateGame(g, _nrows, _ncols);
            gameActive = 0;
            showGameOver(true);
        }
        else if (gameActive == 2)
        {
            try
            {
                GameOfLife g(_nrows, _ncols);
                g.initialize({1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0});
                updateGame(g, _nrows, _ncols);
                gameActive = 0;
                showGameOver(true);
            }
            catch (std::exception& e)
            {
                gameActive = 0;
                showGameOver(false);
            }
        }
        else
        {
            drawStartScreen();
        }
        _window.display();
    }
}

void Screen::handleEvents(char& gameActive)
{
    sf::Event event;
    while (_window.pollEvent(event))
    {
        if (event.type == sf::Event::Closed)
        {
            _window.close();
        }
        if (startButtonsActive)
        {
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left)
            {
                if (isButtonClicked(randomButton, event))
                {
                    gameActive = 1;
                }
                else if (isButtonClicked(exitButton, event))
                {
                    _window.close();
                }
                else if (isButtonClicked(inputButton, event))
                {
                    gameActive = 2;
                }
            }
        }
    }
}

void Screen::updateGame(GameOfLife& g, int rows, int cols)
{
    sf::Clock clock;
    const float updateInterval = 1.0f;
    while (true)
    {
        if (!startButtonsActive && sf::Mouse::isButtonPressed(sf::Mouse::Left))
        {
            if (stopButton.getGlobalBounds().contains((sf::Vector2f)sf::Mouse::getPosition(_window)))
            {
                drawStartScreen();
                return;
            }
        }
        if (clock.getElapsedTime().asSeconds() >= updateInterval)
        {
            if (!g.step())
            {
                break;
            }
            clock.restart();
        }
        _window.clear();

        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                // boring
                // sf::CircleShape cell(cellsize / 2.1);
                // cell.setPosition(col * cellsize, row * cellsize);
                // cell.setFillColor(g.is_alive(row, col) ? sf::Color::White : sf::Color::Black);
                //_window.draw(cell);

                cellSprite.setScale(sf::Vector2f(cellsize / cellSprite.getLocalBounds().width * 1.5,
                                                 cellsize / cellSprite.getLocalBounds().height * 1.5));
                cellSprite.setPosition(col * cellsize, row * cellsize);
                if (g.is_alive(row, col))
                {
                    _window.draw(cellSprite);
                }
            }
        }
        _window.draw(stopButton);
        _window.draw(stopButtonText);
        _window.display();
    }
}

void Screen::initializeButtons()
{
    auto figure = sf::Vector2f(400, 100);

    randomButton.setSize(figure);
    randomButton.setPosition(500, 100);
    randomButton.setFillColor(sf::Color::White);

    exitButton.setSize(figure);
    exitButton.setPosition(500, 500);
    exitButton.setFillColor(sf::Color::White);

    inputButton.setSize(figure);
    inputButton.setPosition(500, 300);
    inputButton.setFillColor(sf::Color::White);

    stopButton.setSize(sf::Vector2f(30, 30));
    stopButton.setPosition(8, 8);
    stopButton.setFillColor(sf::Color(100, 0, 0));

    randomButtonText.setFont(font);
    randomButtonText.setString("random");
    randomButtonText.setCharacterSize(50);
    randomButtonText.setFillColor(sf::Color::Black);
    randomButtonText.setPosition(500 + 130, 100 + 15);

    exitButtonText.setFont(font);
    exitButtonText.setString("exit");
    exitButtonText.setCharacterSize(50);
    exitButtonText.setFillColor(sf::Color::Black);
    exitButtonText.setPosition(500 + 150, 500 + 15);

    inputButtonText.setFont(font);
    inputButtonText.setString("input");
    inputButtonText.setCharacterSize(50);
    inputButtonText.setFillColor(sf::Color::Black);
    inputButtonText.setPosition(500 + 150, 300 + 15);

    stopButtonText.setFont(font);
    stopButtonText.setString("x");
    stopButtonText.setCharacterSize(40);
    stopButtonText.setFillColor(sf::Color::White);
    stopButtonText.setPosition(14, -7);
}

void Screen::drawStartScreen()
{
    startButtonsActive = true;
    _window.draw(randomButton);
    _window.draw(randomButtonText);
    _window.draw(exitButton);
    _window.draw(exitButtonText);
    _window.draw(inputButton);
    _window.draw(inputButtonText);
}

void Screen::drawGameOver(bool what)
{
    sf::CircleShape gameOverShape(100);
    gameOverShape.setFillColor(sf::Color::Red);
    gameOverShape.setPosition(_window.getSize().x / 2.f - 100, _window.getSize().y / 2.f - 100);

    sf::Text gameOverText;
    gameOverText.setFont(font);
    if (what)
    {
        gameOverText.setString("ALL DEAD");
    }
    else
    {
        gameOverText.setString("INPUT-huinia");
    }
    gameOverText.setCharacterSize(90);
    gameOverText.setFillColor(sf::Color::White);

    sf::FloatRect textRect = gameOverText.getLocalBounds();
    gameOverText.setOrigin(textRect.width / 2, textRect.height / 2);
    gameOverText.setPosition(_window.getSize().x / 2.f, _window.getSize().y / 2.f - 40);

    _window.draw(gameOverShape);
    _window.draw(gameOverText);
}

void Screen::showGameOver(bool what)
{
    _window.clear();
    drawGameOver(what);
    _window.display();
    sf::sleep(sf::seconds(2));
}

bool Screen::isButtonClicked(const sf::RectangleShape& Button, const sf::Event& event)
{
    return Button.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y);
}

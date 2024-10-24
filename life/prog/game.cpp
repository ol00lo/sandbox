#include "game.hpp"

GameOfLife::GameOfLife(int height, int width, char typeboard)
    : _nrows(height), _ncols(width), _old_board(_nrows, _ncols, typeboard), _new_board(_nrows, _ncols, typeboard)
{
    if (_nrows < 1 || _ncols < 1)
    {
        throw std::runtime_error("incorrect input");
    }
}

GameOfLife::GameOfLife(Arguments arg) : GameOfLife(arg.height, arg.width, arg.type_board)
{
    arg.validate();
    initialize(std::move(arg.input));
}

bool GameOfLife::step()
{
    for (int i = 0; i < _nrows; i++)
    {
        for (int j = 0; j < _ncols; j++)
        {
            int nneigh = count_of_neighbors(i, j);
            if (_old_board.at(i, j) && nneigh != 2 && nneigh != 3)
            {
                _new_board.set_at(i, j, false);
            }
            else if (!_old_board.at(i, j) && nneigh == 3)
            {
                _new_board.set_at(i, j, true);
            }
        }
    }
    if (is_over())
    {
        return false;
    }
    _old_board = _new_board;
    return true;
}

void GameOfLife::initialize(std::vector<bool>&& in)
{
    if (in.size() != _nrows * _ncols)
    {
        throw std::runtime_error("incorrect input");
    }
    _new_board.add_data(std::move(in));
    _old_board = _new_board;
}

bool GameOfLife::is_over() const
{
    bool diff = (_new_board == _old_board);
    bool life = _new_board.is_alldead();

    if (diff || !life)
        return true;
    else
        return false;
}

bool GameOfLife::is_alive(int row, int col) const
{
    if (row >= _nrows || row < 0 || col >= _ncols || col < 0)
    {
        throw std::runtime_error("out of range");
    }
    return _new_board.at(row, col);
}

int GameOfLife::count_of_neighbors(int row, int col) const
{
    int res = 0;
    if (_old_board.at(row, col - 1))
        res++;
    if (_old_board.at(row - 1, col))
        res++;
    if (_old_board.at(row, col + 1))
        res++;
    if (_old_board.at(row + 1, col))
        res++;
    if (_old_board.at(row - 1, col - 1))
        res++;
    if (_old_board.at(row + 1, col - 1))
        res++;
    if (_old_board.at(row - 1, col + 1))
        res++;
    if (_old_board.at(row + 1, col + 1))
        res++;

    return res;
}

void GameOfLife::display() const
{
    std::cout << "\033[2J\033[H";

    std::cout << " ";
    for (int j = 0; j <= _ncols; ++j)
    {
        std::cout << "_";
    }
    std::cout << "\n";

    for (int i = 0; i < _nrows; ++i)
    {
        std::cout << "|";
        for (int j = 0; j < _ncols; ++j)
        {
            if (is_alive(i, j))
            {
                std::cout << "#";
            }
            else
            {
                std::cout << " ";
            }
        }
        std::cout << " |\n";
    }

    std::cout << " ";
    for (int j = 0; j <= _ncols; ++j)
    {
        std::cout << "-";
    }
    std::cout << "\n";
}

void GameOfLife::over() const
{
    std::cout << "\033[H";
    std::cout << " ";
    for (int j = 0; j <= _ncols; ++j)
    {
        std::cout << "_";
    }
    std::cout << "\n";
    for (int i = 0; i < _nrows; ++i)
    {
        std::cout << "|";
        if (i == _nrows / 2)
        {
            std::cout << std::setw(_ncols) << "ALLDEAD";
        }
        else
        {
            for (int j = 0; j < _ncols; ++j)
            {
                std::cout << " ";
            }
        }
        std::cout << " |\n";
    }
    std::cout << " ";
    for (int j = 0; j <= _ncols; ++j)
    {
        std::cout << "-";
    }
    std::cout << "\n";
}
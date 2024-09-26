#include "game.hpp"
std::random_device rd;
std::mt19937 ran(rd());

GameOfLife::GameOfLife(int height, int width)
    : _nrows(height), _ncols(width), _old_board(height * width, false), _new_board(height * width, false)
{
    if (height < 1 || width < 1)
    {
        throw std::runtime_error("incorrect input");
    }
}

bool GameOfLife::step()
{
    for (int i = 0; i < _nrows; i++)
    {
        for (int j = 0; j < _ncols; j++)
        {
            int nneigh = count_of_neighbors(i, j);
            if (_old_board[i * _ncols + j] && nneigh != 2 && nneigh != 3)
            {
                _new_board[i * _ncols + j] = false;
            }
            else if (!_old_board[i * _ncols + j] && nneigh == 3)
            {
                _new_board[i * _ncols + j] = true;
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

void GameOfLife::initialize(const std::vector<bool>&& in)
{
    if (in.size() != _nrows * _ncols)
    {
        throw std::runtime_error("incorrect input");
    }
    _new_board = in;
    _old_board = in;
}

bool GameOfLife::is_over() const
{
    bool life = false;
    bool diff = false;
    for (int i = 0; i < _nrows; i++)
    {
        for (int j = 0; j < _ncols; j++)
        {
            if (_new_board[i * _ncols + j] != _old_board[i * _ncols + j] && diff == false)
            {
                diff = true;
            }
            if (_new_board[i * _ncols + j] && life == false)
            {
                life = true;
            }
            if (life && diff)
                break;
        }
        if (life && diff)
            break;
    }
    if (diff == false || life == false)
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
    return _new_board[row * _ncols + col];
}

int GameOfLife::count_of_neighbors(int row, int col) const
{
    int res = 0;
    res += (col > 0) ? int(_old_board[row * _ncols + col - 1]) : 0;
    res += (row > 0) ? int(_old_board[(row - 1) * _ncols + col]) : 0;
    res += (col < _ncols - 1) ? int(_old_board[row * _ncols + col + 1]) : 0;
    res += (row < _nrows - 1) ? int(_old_board[(row + 1) * _ncols + col]) : 0;
    res += (row > 0 && col > 0) ? int(_old_board[(row - 1) * _ncols + col - 1]) : 0;
    res += (col > 0 && row < _nrows - 1) ? int(_old_board[(row + 1) * _ncols + col - 1]) : 0;
    res += (row > 0 && col < _ncols - 1) ? int(_old_board[(row - 1) * _ncols + col + 1]) : 0;
    res += (col < _ncols - 1 && row < _nrows - 1) ? int(_old_board[(row + 1) * _ncols + col + 1]) : 0;

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

void GameOfLife::random_initialize(int norganisms)
{
    std::uniform_int_distribution row(0, _nrows - 1);
    std::uniform_int_distribution col(0, _ncols - 1);
    for (int i = 0; i < norganisms; i++)
    {
        int irow = row(ran);
        int icol = col(ran);
        _old_board[irow * _ncols + icol] = true;
        _new_board[irow * _ncols + icol] = true;
    }
}
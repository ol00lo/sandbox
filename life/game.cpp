#include "game.hpp"
std::random_device rd;
std::mt19937 ran(rd());

GameOfLife::GameOfLife(int height, int width)
    : _nrows(height), _ncols(width), _old_board(_nrows, _ncols), _new_board(_nrows, _ncols)
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
    if (col > 0 && _old_board.at(row, col - 1))
        res++;
    if (row > 0 && _old_board.at(row - 1, col))
        res++;
    if (col < _ncols - 1 && _old_board.at(row, col + 1))
        res++;
    if (row < _nrows - 1 && _old_board.at(row + 1, col))
        res++;
    if (row > 0 && col > 0 && _old_board.at(row - 1, col - 1))
        res++;
    if (col > 0 && row < _nrows - 1 && _old_board.at(row + 1, col - 1))
        res++;
    if (row > 0 && col < _ncols - 1 && _old_board.at(row - 1, col + 1))
        res++;
    if (col < _ncols - 1 && row < _nrows - 1 && _old_board.at(row + 1, col + 1))
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

void GameOfLife::random_initialize(int norganisms)
{
    std::uniform_int_distribution ind(0, _nrows * _ncols - 1);
    for (int i = 0; i < norganisms; i++)
    {
        int at = ind(ran);
        _new_board.set_at(at, true);
    }
    _old_board = _new_board;
}

Board::Board(int nrows, int ncols) : _nrows(nrows), _ncols(ncols), _board(nrows * ncols, false)
{
}

void Board::operator=(const Board& other)
{
    if (_nrows != other._nrows || _ncols != other._ncols)
    {
        throw std::out_of_range("different dimensions");
    }
    _board = other._board;
}

bool Board::operator==(const Board& other) const
{
    return _nrows == other._nrows && _ncols == other._ncols && _board == other._board;
}

void Board::add_data(std::vector<bool>&& in)
{
    if (_board.size() != in.size())
    {
        throw std::out_of_range("different dimensions");
    }
    _board = in;
}

bool Board::at(int irow, int icol) const
{
    if (irow >= _nrows || irow < 0 || icol >= _ncols || icol < 0)
    {
        throw std::runtime_error("out of range");
    }
    return _board[irow * _ncols + icol];
}

void Board::set_at(int irow, int icol, bool val)
{
    if (irow >= _nrows || irow < 0 || icol >= _ncols || icol < 0)
    {
        throw std::runtime_error("out of range");
    }
    _board[irow * _ncols + icol] = val;
}

void Board::set_at(int ind, bool val)
{
    if (ind >= _nrows * _ncols || ind < 0)
    {
        throw std::runtime_error("out of range");
    }
    _board[ind] = val;
}

bool Board::is_alldead() const
{
    return std::any_of(_board.begin(), _board.end(), [](bool value) { return !value; });
}
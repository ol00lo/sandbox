#include "board.hpp"

Board::Board(int nrows, int ncols, char boardtype)
    : _nrows(nrows), _ncols(ncols), _board(nrows * ncols, false), type_of_board(boardtype)
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
    _board = std::move(in);
}

bool Board::at(int irow, int icol) const
{
    switch (type_of_board)
    {   
    case 'w':
        if (irow >= _nrows || irow < 0 || icol >= _ncols || icol < 0)
        {
            return false;
        }
        break;
    case 'p':
        if (irow == -1)
            irow = _nrows - 1;
        if (icol == -1)
            icol = _ncols - 1;
        if (irow == _nrows)
            irow = 0;
        if (icol == _ncols)
            icol = 0;
        break;
    case 'm':
        if (irow == -1)
            irow = 0;
        if (icol == -1)
            icol = 0;
        if (irow == _nrows)
            irow = _nrows - 1;
        if (icol == _ncols)
            icol = _ncols - 1;
        break;
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
#include "cmd_viewer.hpp"
#include <iomanip>

void CmdViewer::display(const Board& board)
{
    _ncols = board.ncols();
    _nrows = board.nrows();
    std::cout << "\033[2J\033[H";

    std::cout << " ";
    for (int j = 0; j <= board.ncols(); ++j)
    {
        std::cout << "_";
    }
    std::cout << "\n";

    for (int i = 0; i < board.nrows(); ++i)
    {
        std::cout << "|";
        for (int j = 0; j < board.ncols(); ++j)
        {
            if (is_alive(board, i, j))
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
    for (int j = 0; j <= board.ncols(); ++j)
    {
        std::cout << "-";
    }
    std::cout << "\n";
}

void CmdViewer::game_over()
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

bool CmdViewer::is_alive(const Board& board, int row, int col) const
{
    if (row >= board.nrows() || row < 0 || col >= board.ncols() || col < 0)
    {
        throw std::runtime_error("out of range");
    }
    return board.at(row, col);
}

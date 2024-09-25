#include "game.hpp"
#include <iomanip>

Game::Game(int Height, int Width) : nrows(Height), ncols(Width)
{
    new_board.resize(nrows, std::vector<bool>(ncols, false));
    old_board.resize(nrows, std::vector<bool>(ncols, false));
}

bool Game::step()
{
    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < ncols; j++)
        {
            int nneigh = count_of_neighbors(i, j);
            if (old_board[i][j] && nneigh != 2 && nneigh != 3)
            {
                new_board[i][j] = false;
            }
            else if (!old_board[i][j] && nneigh == 3)
            {
                new_board[i][j] = true;
            }
        }
    }
    if (is_over())
    {
        return false;
    }
    old_board = new_board;
    return true;
}

void Game::initialize(std::vector<std::vector<bool>> in)
{
    if (in.size() != nrows && in[0].size() != ncols)
    {
        throw std::runtime_error("incorrect input");
    }
    new_board = in;
    old_board = in;
}

bool Game::is_over() const
{
    bool life = false;
    bool diff = false;
    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < ncols; j++)
        {
            if (new_board[i][j] != old_board[i][j])
            {
                diff = true;
            }
            if (new_board[i][j])
            {
                life = true;
            }
        }
    }
    if (diff == false || life == false)
        return true;
    else
        return false;
}

bool Game::is_alive(int row, int col) const
{
    if (row >= nrows || row < 0 || col >= ncols || col < 0)
    {
        throw std::runtime_error("out of range");
    }
    return new_board[row][col];
}

int Game::count_of_neighbors(int row, int col) const
{
    int res = 0;
    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
            if (i == 0 && j == 0)
            {
                continue;
            }
            int irow = row + i;
            int jcol = col + j;
            if (irow >= 0 && jcol >= 0)
            {
                if (irow < nrows && jcol < ncols)
                {
                    if (old_board[irow][jcol] == true)
                    {
                        res++;
                    }
                }
            }
        }
    }
    return res;
}

void Game::display() const
{
    std::cout << "\033[H";

    std::cout << " ";
    for (int j = 0; j <= ncols; ++j)
    {
        std::cout << "_";
    }
    std::cout << "\n";

    for (int i = 0; i < nrows; ++i)
    {
        std::cout << "|";
        for (int j = 0; j < ncols; ++j)
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
    for (int j = 0; j <= ncols; ++j)
    {
        std::cout << "-";
    }
}
void Game::over() const
{
    std::cout << "\033[H";

    std::cout << " ";
    for (int j = 0; j <= ncols; ++j)
    {
        std::cout << "_";
    }
    std::cout << "\n";

    for (int i = 0; i < nrows; ++i)
    {
        std::cout << "|";
        if (i == nrows / 2)
        {
            std::cout<<std::setw(ncols) << "ALLDEAD";
        }
        else
        {
            for (int j = 0; j < ncols; ++j)
            {
                std::cout << " ";
            }
        }

        std::cout << " |\n";
    }

    std::cout << " ";
    for (int j = 0; j <= ncols; ++j)
    {
        std::cout << "-";
    }
}
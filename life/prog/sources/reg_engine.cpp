#include "reg_engine.hpp"

Board reg_engine::step(const Board& board) const
{
    Board new_board(board);
    for (int i = 0; i < board.nrows(); i++)
    {
        for (int j = 0; j < board.ncols(); j++)
        {
            int nneigh = count_of_neighbors(board, i, j);
            if (board.at(i, j) && nneigh != 2 && nneigh != 3)
            {
                new_board.set_at(i, j, false);
            }
            else if (!board.at(i, j) && nneigh == 3)
            {
                new_board.set_at(i, j, true);
            }
        }
    }
    return new_board;
}

int reg_engine::count_of_neighbors(const Board& board, int row, int col) const
{
    int res = 0;
    if (board.at(row, col - 1))
        res++;
    if (board.at(row - 1, col))
        res++;
    if (board.at(row, col + 1))
        res++;
    if (board.at(row + 1, col))
        res++;
    if (board.at(row - 1, col - 1))
        res++;
    if (board.at(row + 1, col - 1))
        res++;
    if (board.at(row - 1, col + 1))
        res++;
    if (board.at(row + 1, col + 1))
        res++;
    return res;
}

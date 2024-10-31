#include "board.h"
#include <stdbool.h>
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>

Board* create_board(int nrows, int ncols, char boardtype)
{
    Board* new_board = (Board*)malloc(sizeof(Board));
    if (!new_board)
    {
        return NULL; 
    }
    new_board->nrows = nrows;
    new_board->ncols = ncols;
    new_board->type_of_board = boardtype;

    new_board->board = (bool*)calloc(nrows * ncols, sizeof(bool)); 
    if (!new_board->board)
    {
        free(new_board);
        return NULL; 
    }

    return new_board;
}

void destroy_board(Board* board)
{
    if (board)
    {
        free(board->board);
        free(board);
    }
}

void board_add_data(Board* board, bool* in, int size)
{
    if (board->nrows * board->ncols != size)
    {
        fprintf(stderr, "Error: different dimensions\n");
        return; 
    }
    memcpy(board->board, in, size * sizeof(bool));
}

bool board_at(const Board* board, int irow, int icol)
{
    switch (board->type_of_board)
    {
    case 'w':
        if (irow < 0 || irow >= board->nrows || icol < 0 || icol >= board->ncols)
        {
            return false;
        }
        break;
    case 'p':
        if (irow == -1)
            irow = board->nrows - 1;
        if (icol == -1)
            icol = board->ncols - 1;
        if (irow == board->nrows)
            irow = 0;
        if (icol == board->ncols)
            icol = 0;
        break;
    case 'm':
        if (irow < 0)
            irow = 0;
        if (icol < 0)
            icol = 0;
        if (irow >= board->nrows)
            irow = board->nrows - 1;
        if (icol >= board->ncols)
            icol = board->ncols - 1;
        break;
    default:
        fprintf(stderr, "Error: unreachable code\n");
        return false; 
    }
    return board->board[irow * board->ncols + icol];
}

void board_set_at(Board* board, int irow, int icol, bool val)
{
    if (irow < 0 || irow >= board->nrows || icol < 0 || icol >= board->ncols)
    {
        fprintf(stderr, "Error: out of range\n");
        return; 
    }
    board->board[irow * board->ncols + icol] = val;
}

void board_set_at_index(Board* board, int ind, bool val)
{
    if (ind < 0 || ind >= board->nrows * board->ncols)
    {
        fprintf(stderr, "Error: out of range\n");
        return; 
    }
    board->board[ind] = val;
}
bool board_is_alldead(const Board* board)
{
    for (int i = 0; i < board->nrows * board->ncols; i++)
    {
        if (board->board[i])
        {
            return false; 
        }
    }
    return true; 
}

int board_nrows(const Board* board)
{
    return board->nrows;
}

int board_ncols(const Board* board)
{
    return board->ncols;
}

bool* board_get_board(const Board* board)
{
    return board->board;
}
#include "cboard.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Board* board_create(int nrows, int ncols, char boardtype)
{
    Board* board = (Board*)malloc(sizeof(Board));
    board->nrows = nrows;
    board->ncols = ncols;
    board->type_of_board = boardtype;
    board->board = (bool*)calloc(nrows * ncols, sizeof(bool)); 
    return board;
}

void board_destroy(Board* board)
{
    if (board)
    {
        free(board->board);
        free(board);
    }
}

void board_copy(Board* dest, const Board* src)
{
    if (dest->nrows != src->nrows || dest->ncols != src->ncols)
    {
        fprintf(stderr, "different dimensions\n");
        return; 
    }
    memcpy(dest->board, src->board, sizeof(bool) * (src->nrows * src->ncols));
}

bool board_equals(const Board* board1, const Board* board2)
{
    return board1->nrows == board2->nrows && board1->ncols == board2->ncols &&
           memcmp(board1->board, board2->board, sizeof(bool) * (board1->nrows * board1->ncols)) == 0;
}

void board_add_data(Board* board, bool* in, int size)
{
    if ((board->nrows * board->ncols) != size)
    {
        fprintf(stderr, "different dimensions\n");
        return; 
    }
    memcpy(board->board, in, sizeof(bool) * size);
}

bool board_at(const Board* board, int irow, int icol)
{
    switch (board->type_of_board)
    {
    case 'w':
        if (irow >= board->nrows || irow < 0 || icol >= board->ncols || icol < 0)
            return false;
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
        if (irow == -1)
            irow = 0;
        if (icol == -1)
            icol = 0;
        if (irow == board->nrows)
            irow = board->nrows - 1;
        if (icol == board->ncols)
            icol = board->ncols - 1;
        break;
    default:
        fprintf(stderr, "unreachable\n");
        return false; 
    }
    return board->board[irow * board->ncols + icol];
}

void board_set_at(Board* board, int irow, int icol, bool val)
{
    if (irow >= board->nrows || irow < 0 || icol >= board->ncols || icol < 0)
    {
        fprintf(stderr, "out of range\n");
        return; 
    }
    board->board[irow * board->ncols + icol] = val;
}

void board_set_at_index(Board* board, int index, bool val)
{
    if (index >= board->nrows * board->ncols || index < 0)
    {
        fprintf(stderr, "out of range\n");
        return;
    }
    board->board[index] = val;
}

bool board_is_alldead(const Board* board)
{
    for (int i = 0; i < board->nrows * board->ncols; i++)
    {
        if (board->board[i] == true)
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
    bool* copy = (bool*)malloc(sizeof(bool) * (board->nrows * board->ncols));
    memcpy(copy, board->board, sizeof(bool) * (board->nrows * board->ncols));
    return copy; 
}
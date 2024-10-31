#include "game.h"
#include <stdio.h>  
#include <stdlib.h> 
#include <string.h> 

void reg_engine_step(const Engine* engine, const Board* board, Board* new_board);
int count_of_neighbors(const Board* board, int row, int col);

void reg_engine_step(const Engine* engine, const Board* board, Board* new_board)
{
    for (int i = 0; i < board_nrows(board); i++)
    {
        for (int j = 0; j < board_ncols(board); j++)
        {
            int nneigh = count_of_neighbors(board, i, j);
            if (board_at(board, i, j) && nneigh != 2 && nneigh != 3)
            {
                board_set_at(new_board, i, j, false);
            }
            else if (!board_at(board, i, j) && nneigh == 3)
            {
                board_set_at(new_board, i, j, true);
            }
        }
    }
}

int count_of_neighbors(const Board* board, int row, int col)
{
    int res = 0;
    if (col > 0 && board_at(board, row, col - 1))
        res++;
    if (row > 0 && board_at(board, row - 1, col))
        res++;
    if (col < board_ncols(board) - 1 && board_at(board, row, col + 1))
        res++;
    if (row < board_nrows(board) - 1 && board_at(board, row + 1, col))
        res++;
    if (row > 0 && col > 0 && board_at(board, row - 1, col - 1))
        res++;
    if (row < board_nrows(board) - 1 && col > 0 && board_at(board, row + 1, col - 1))
        res++;
    if (row > 0 && col < board_ncols(board) - 1 && board_at(board, row - 1, col + 1))
        res++;
    if (row < board_nrows(board) - 1 && col < board_ncols(board) - 1 && board_at(board, row + 1, col + 1))
        res++;
    return res;
}

void cmd_viewer_display(const Viewer* viewer, const Board* board)
{
    printf("\033[2J\033[H");
    int nrows = board_nrows(board);
    int ncols = board_ncols(board);

    printf(" ");
    for (int j = 0; j < ncols; ++j)
    {
        printf("_");
    }
    printf("\n");

    for (int i = 0; i < nrows; ++i)
    {
        printf("|");
        for (int j = 0; j < ncols; ++j)
        {
            printf(board_at(board, i, j) ? "#" : " ");
        }
        printf(" |\n");
    }

    printf(" ");
    for (int j = 0; j < ncols; ++j)
    {
        printf("-");
    }
    printf("\n");
}

void cmd_viewer_game_over(const Viewer* viewer)
{
    printf("\033[H");
    printf(" ");
    for (int j = 0; j < viewer->_ncols; ++j)
    {
        printf("_");
    }
    printf("\n");
    for (int i = 0; i < viewer->_nrows; ++i)
    {
        printf("|");
        if (i == viewer->_nrows / 2)
        {
            printf("ALLDEAD");
        }
        else
        {
            for (int j = 0; j < viewer->_ncols; ++j)
            {
                printf(" ");
            }
        }
        printf(" |\n");
    }
    printf(" ");
    for (int j = 0; j < viewer->_ncols; ++j)
    {
        printf("-");
    }
    printf("\n");
}

void initialize_game(Driver* driver)
{
    driver->board = create_board(5, 3, 'w'); 
    bool* in = (bool*)malloc(15 * sizeof(bool));
    if (in != NULL)
    {
        in[0] = 1;
        in[1] = 0;
        in[2] = 1;
        in[3] = 0;
        in[4] = 1;
        in[5] = 0;
        in[6] = 1;
        in[7] = 1;
        in[8] = 0;
        in[9] = 0;
        in[10] = 0;
        in[11] = 1;
        in[12] = 1;
        in[13] = 0;
        in[14] = 1;
    }
    board_add_data(driver->board, in, 15);
    driver->viewer = (Viewer*)malloc(sizeof(Viewer));
    driver->engine = (Engine*)malloc(sizeof(Engine));

    driver->viewer->interface = (ViewerInterface*)malloc(sizeof(ViewerInterface));
    driver->engine->interface = (EngineInterface*)malloc(sizeof(EngineInterface));

    driver->viewer->interface->display = cmd_viewer_display;
    driver->viewer->interface->game_over = cmd_viewer_game_over;

    driver->viewer->_nrows = board_nrows(driver->board);
    driver->viewer->_ncols = board_ncols(driver->board);

    driver->engine->interface->step = reg_engine_step;
}

void cleanup_game(Driver* driver)
{
    destroy_board(driver->board);   
    free(driver->viewer->interface);
    free(driver->viewer);           
    free(driver->engine->interface);
    free(driver->engine);           
}

bool board_compare(const Board* board1, const Board* board2)
{
    if (board_nrows(board1) != board_nrows(board2) || board_ncols(board1) != board_ncols(board2))
    {
        return false;
    }
    for (int i = 0; i < board_nrows(board1); i++)
    {
        for (int j = 0; j < board_ncols(board1); j++)
        {
            if (board_at(board1, i, j) != board_at(board2, i, j))
            {
                return false;
            }
        }
    }
    return true;
}

bool is_over(Driver* driver, Board* new_board)
{
    bool diff = board_compare(driver->board, new_board);
    bool life = board_is_alldead(new_board);

    return diff || life;
}
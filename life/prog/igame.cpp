#include "igame.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct Engine
{
};

struct Viewer
{
    int _ncols;
    int _nrows;
};

struct Driver
{
    Arguments arg;
    Board board;
    Engine* engine;
    Viewer* viewer;
};

Engine* engine_create()
{
    Engine* engine = (Engine*)malloc(sizeof(Engine));
    return engine;
}

void engine_destroy(Engine* engine)
{
    free(engine);
}

Board engine_step(Engine* engine, const Board* board)
{
    Board new_board = *board;
    for (int i = 0; i < board->nrows(); i++)
    {
        for (int j = 0; j < board->ncols(); j++)
        {
            int nneigh = 0;
            if (board->at(i, j - 1))
                nneigh++;
            if (board->at(i - 1, j))
                nneigh++;
            if (board->at(i, j + 1))
                nneigh++;
            if (board->at(i + 1, j))
                nneigh++;
            if (board->at(i - 1, j - 1))
                nneigh++;
            if (board->at(i + 1, j - 1))
                nneigh++;
            if (board->at(i - 1, j + 1))
                nneigh++;
            if (board->at(i + 1, j + 1))
                nneigh++;
            //int nneigh = count_of_neighbors(board, i, j);
            if (board->at(i, j) && nneigh != 2 && nneigh != 3)
            {
                new_board.set_at(i, j, false);
            }
            else if (!board->at(i, j) && nneigh == 3)
            {
                new_board.set_at(i, j, true);
            }
        }
    }
    return new_board;
}

int count_of_neighbors(const Board* board, int row, int col)
{
    int res = 0;
    if (board->at(row, col - 1))
        res++;
    if (board->at(row - 1, col))
        res++;
    if (board->at(row, col + 1))
        res++;
    if (board->at(row + 1, col))
        res++;
    if (board->at(row - 1, col - 1))
        res++;
    if (board->at(row + 1, col - 1))
        res++;
    if (board->at(row - 1, col + 1))
        res++;
    if (board->at(row + 1, col + 1))
        res++;
    return res;
}

Viewer* viewer_create()
{
    Viewer* viewer = (Viewer*)malloc(sizeof(Viewer));
    return viewer;
}

void viewer_destroy(Viewer* viewer)
{
    free(viewer);
}

void viewer_display(Viewer* viewer, const Board* board)
{
    viewer->_ncols = board->ncols();
    viewer->_nrows = board->nrows();
    printf("\033[2J\033[H");

    printf(" ");
    for (int j = 0; j <= board->ncols(); ++j)
    {
        printf("_");
    }
    printf("\n");

    for (int i = 0; i < board->nrows(); ++i)
    {
        printf("|");
        for (int j = 0; j < board->ncols(); ++j)
        {
            if (i >= board->nrows() || i < 0 || j >= board->ncols() || j < 0)
            {
                fprintf(stderr, "out of range\n");
                exit(EXIT_FAILURE);
            }
            bool is_alive = board->at(i, j);
            //if (is_alive(board, i, j))
            if (is_alive)
            {
                printf("#");
            }
            else
            {
                printf(" ");
            }
        }
        printf(" |\n");
    }

    printf(" ");
    for (int j = 0; j <= board->ncols(); ++j)
    {
        printf("-");
    }
    printf("\n");
}

void viewer_game_over(Viewer* viewer)
{
    printf("\033[H");
    printf(" ");
    for (int j = 0; j <= viewer->_ncols; ++j)
    {
        printf("_");
    }
    printf("\n");
    for (int i = 0; i < viewer->_nrows; ++i)
    {
        printf("|");
        if (i == viewer->_nrows / 2)
        {
            printf("%*s", viewer->_ncols, "ALLDEAD");
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
    for (int j = 0; j <= viewer->_ncols; ++j)
    {
        printf("-");
    }
    printf("\n");
}

bool is_alive(const Board* board, int row, int col)
{
    if (row >= board->nrows() || row < 0 || col >= board->ncols() || col < 0)
    {
        fprintf(stderr, "out of range\n");
        exit(EXIT_FAILURE);
    }
    return board->at(row, col);
}
Board create_board(int height, int width, char type_board)
{
    return Board(height, width, type_board);
}
Driver* driver_create(const Arguments* arg)
{
    Driver* driver = (Driver*)malloc(sizeof(Driver));
    driver->arg = *arg;
    driver->board = create_board(arg->height, arg->width, arg->type_board);

    driver->engine = engine_create();
    driver->viewer = viewer_create();
    return driver;
}

void driver_destroy(Driver* driver)
{
    engine_destroy(driver->engine);
    viewer_destroy(driver->viewer);
    free(driver);
}

void driver_start(Driver* driver)
{
    viewer_display(driver->viewer, &driver->board);
    while (driver_next(driver))
    {
        viewer_display(driver->viewer, &driver->board);
        //usleep(driver->arg.delay * 1000); 
    }
    viewer_game_over(driver->viewer);
}

int driver_next(Driver* driver)
{
    Board new_board = engine_step(driver->engine, &driver->board);
    if (driver_is_over(driver, &new_board))
    {
        return 0; 
    }
    else
    {
        driver->board = new_board;
        return 1; 
    }
}

int driver_is_over(Driver* driver, const Board* new_board)
{
    bool diff = memcmp(new_board, &driver->board, sizeof(Board)) == 0;
    bool life = new_board->is_alldead();

    if (diff || !life)
    {
        return 1; 
    }
    else
    {
        return 0; 
    }
}
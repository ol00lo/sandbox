#include "game.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

void sleep_ms(int milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds * 1000);
#endif
}

void reg_engine_step(const Engine* engine, const Board* board, Board* new_board);
int count_of_neighbors(const Board* board, int row, int col);
bool board_compare(const Board* board1, const Board* board2);
bool is_over(Driver* driver, Board* new_board);
bool board_compare(const Board* board1, const Board* board2);
void cleanup_game(Driver* driver);

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

void initialize_game(Driver* driver, Arguments& arg)
{
    driver->board = create_board(arg.height, arg.width, arg.type_board);
    bool* in = (bool*)malloc(arg.height * arg.width * sizeof(bool));
    for (int i = 0; i < arg.height * arg.width; i++)
    {
        in[i] = arg.input[i];
    }
    driver->delay = arg.delay;
    board_add_data(driver->board, in, arg.height * arg.width);
    driver->viewer = (Viewer*)malloc(sizeof(Viewer));
    driver->viewer->inter = (ViewerInterface*)malloc(sizeof(ViewerInterface));
    driver->viewer->inter->display = cmd_viewer_display;
    driver->viewer->inter->game_over = cmd_viewer_game_over;
    driver->viewer->_nrows = board_nrows(driver->board);
    driver->viewer->_ncols = board_ncols(driver->board);

    driver->engine = (Engine*)malloc(sizeof(Engine));
    driver->engine->inter = (EngineInterface*)malloc(sizeof(EngineInterface));
    driver->engine->inter->step = reg_engine_step;
}

void cleanup_game(Driver* driver)
{
    destroy_board(driver->board);
    free(driver->viewer->inter);
    free(driver->viewer);
    free(driver->engine->inter);
    free(driver->engine);
}

bool board_compare(const Board* board1, const Board* board2)
{
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

void run_game(Driver driver)
{
    // Board* new_board = create_board(board_nrows(driver.board), board_ncols(driver.board), 'w');
    // board_add_data_from_other(new_board, driver.board);

    driver.viewer->inter->display(driver.viewer, driver.board);
    while (1)
    {
        Board* new_board = create_board(board_nrows(driver.board), board_ncols(driver.board), 'w');
        board_add_data_from_other(new_board, driver.board);
        driver.engine->inter->step(driver.engine, driver.board, new_board);

        if (is_over(&driver, new_board))
        {
            driver.viewer->inter->game_over(driver.viewer);
            break;
        }

        driver.viewer->inter->display(driver.viewer, driver.board);
        driver.board->board = new_board->board;
        sleep_ms(driver.delay);
    }
    cleanup_game(&driver);
}
#include "arguments.hpp"
#include "cxxopts.hpp"
#include "game.h"
#include "board.h"
#include <thread>
#include <iostream>
#include <random>
#include <sstream>
#include <string>

void test_board()
{
    Board* b = create_board(5, 3, 'w');
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
    board_add_data(b, in, 15);
    for (int i = 0; i < board_nrows(b); i++)
    {
        for (int j = 0; j < board_ncols(b); j++)
        {
            std::cout << board_at(b, i, j) << "  ";
        }
        std::cout << std::endl;
    }

    destroy_board(b);
    free(in);
}


void test_game()
{
    Driver driver;

    initialize_game(&driver);

    while (1)
    {
        driver.viewer->interface->display(driver.viewer, driver.board);

        Board* new_board = create_board(board_nrows(driver.board), board_ncols(driver.board),'w');
        driver.engine->interface->step(driver.engine, driver.board, new_board);

        if (is_over(&driver, new_board))
        {
            driver.viewer->interface->game_over(driver.viewer);
            break;
        }
        
        driver.board = new_board;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    cleanup_game(&driver);
}

int main(int argc, char* argv[])
{
    try
    {
        // test_board();
        test_game();
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}
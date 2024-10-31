
#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus
    typedef struct
    {
        int nrows;
        int ncols;
        bool* board;
        char type_of_board;
    } Board;

    Board* create_board(int ncols, int nrows, char boardtype);
    void destroy_board(Board* board);
    void board_add_data_from_other(Board* board, Board* other);
    void board_add_data(Board* board, bool* in, int size);
    bool board_at(const Board* board, int irow, int icol);
    void board_set_at(Board* board, int irow, int icol, bool val);
    bool board_is_alldead(const Board* board);
    int board_nrows(const Board* board);
    int board_ncols(const Board* board);
    char board_type(const Board* board);
    bool* board_get_board(const Board* board);
#ifdef __cplusplus
}
#endif // __cplusplus
#endif // BOARD_H
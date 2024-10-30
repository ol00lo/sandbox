#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        int nrows;
        int ncols;
        bool* board; 
        char type_of_board;
    } Board;

    Board* board_create(int nrows, int ncols, char boardtype);
    void board_destroy(Board* board);
    void board_copy(Board* dest, const Board* src);
    bool board_equals(const Board* board1, const Board* board2);
    void board_add_data(Board* board, bool* in, int size);
    bool board_at(const Board* board, int irow, int icol);
    void board_set_at(Board* board, int irow, int icol, bool val);
    void board_set_at_index(Board* board, int index, bool val);
    bool board_is_alldead(const Board* board);
    int board_nrows(const Board* board);
    int board_ncols(const Board* board);
    bool* board_get_board(const Board* board);

#ifdef __cplusplus
}
#endif

#endif // BOARD_H
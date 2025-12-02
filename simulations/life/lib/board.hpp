#ifndef BOARD_HPP
#define BOARD_HPP
#include "simlife_lib.h"
#include <algorithm>
#include <iostream>
#include <vector>

class LIFELIB_API Board
{
public:
    Board(int ncols, int nrows);
    void operator=(const Board& other);
    bool operator==(const Board& other) const;
    void add_data(const std::vector<bool>& in);
    bool at(int irow, int icol) const;
    void set_at(int irow, int icol, bool val);
    void set_at(int ind, bool val);
    bool is_alldead() const;
    int nrows() const;
    int ncols() const;
    std::vector<bool> get_board() const;
    void set_type(char type);

private:
    const int _nrows, _ncols;
    std::vector<bool> _board;
    char _type_of_board = 'w';
};
#endif // !BOARD_HPP
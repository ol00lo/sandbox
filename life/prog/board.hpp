#ifndef BOARD_HPP
#define BOARD_HPP
#include <algorithm>
#include <iostream>
#include <vector>

class Board
{
public:
    Board(int ncols, int nrows);
    void operator=(const Board& other);
    bool operator==(const Board& other) const;
    void add_data(std::vector<bool>&& in);
    bool at(int irow, int icol) const;
    void set_at(int irow, int icol, bool val);
    void set_at(int ind, bool val);
    bool is_alldead() const;

private:
    const int _nrows, _ncols;
    std::vector<bool> _board;
};
#endif // !BOARD_HPP
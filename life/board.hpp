#include <iostream>
#include <vector>
#include <algorithm>

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

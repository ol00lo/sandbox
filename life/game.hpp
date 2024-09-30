#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
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

class GameOfLife
{
public:
    GameOfLife(int height, int width);
    void initialize(std::vector<bool>&& in);
    void random_initialize(int norganisms);
    bool step(); // return "false" if game is over or "true" if step is successful
    bool is_alive(int row, int col) const;
    void display() const;
    void over() const;

private:
    const int _nrows, _ncols;
    Board _new_board;
    Board _old_board;
    int count_of_neighbors(int row, int col) const;
    bool is_over() const;
};
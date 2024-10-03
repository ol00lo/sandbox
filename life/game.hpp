#include <chrono>
#include <iomanip>
#include <random>
#include "board.hpp"

class GameOfLife
{
public:
    GameOfLife(int height, int width);
    void initialize(std::vector<bool>&& in);
    void random_initialize(int norganisms);
    bool step(); // return "false" if game is over or "true" if step is successful
    void display() const;
    void over() const;

private:
    const int _nrows, _ncols;
    Board _new_board;
    Board _old_board;
    int count_of_neighbors(int row, int col) const;
    bool is_alive(int row, int col) const;
    bool is_over() const;
};
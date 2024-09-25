#include <iostream>
#include <vector>

class Game
{
public:
    Game(int Height, int Width);
    void initialize(std::vector<std::vector<bool>> in);
    bool step();
    bool is_alive(int row, int col) const;
    void display() const;
    void over() const;

private:
    int nrows, ncols;
    std::vector<std::vector<bool>> new_board;
    std::vector<std::vector<bool>> old_board;
    int count_of_neighbors(int x, int y) const;
    bool is_over() const;
};
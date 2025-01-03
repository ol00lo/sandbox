#ifndef CMD_VIEWER_HPP
#define CMD_VIEWER_HPP
#include "i_viewer.hpp"

class CmdViewer : public IViewer
{
public:
    void display(const Board& board) override;
    void game_over() override;

private:
    bool is_alive(const Board& board, int row, int col) const;
    int _ncols = 0;
    int _nrows = 0;
};
#endif
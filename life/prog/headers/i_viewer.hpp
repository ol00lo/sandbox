#ifndef VIEWER_HPP
#define VIEWER_HPP
#include "board.hpp"

class IViewer
{
public:
    virtual ~IViewer() = default;
    virtual void display(const Board& board) = 0;
    virtual void game_over() = 0;
};

#endif
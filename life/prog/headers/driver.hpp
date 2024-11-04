#ifndef DRIVER_HPP
#define DRIVER_HPP
#include "arguments.hpp"
#include "i_engine.hpp"
#include "i_viewer.hpp"

class Driver
{
public:
    Driver(const Arguments& arg);
    void start();
    bool is_over(const Board& new_board) const;
    bool next();

private:
    Arguments _arg;
    Board _board;
    std::unique_ptr<IEngine> _engine;
    std::unique_ptr<IViewer> _viewer;
};

#endif
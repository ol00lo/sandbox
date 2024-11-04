#ifndef DRIVER_HPP
#define DRIVER_HPP
#include "arguments.hpp"
#include "engine.hpp"
#include "viewer.hpp"
#include <thread>

class Driver
{
public:
    Driver(const Arguments& arg);
    void start();
    bool is_over(const Board& new_board) const;
    bool next();

private:
    Arguments arg;
    Board board;
    std::unique_ptr<IEngine> engine;
    std::unique_ptr<IViewer> viewer;
};

#endif
#ifndef DRIVER_HPP
#define DRIVER_HPP
#include "i_engine.hpp"
#include "i_viewer.hpp"
#include "simlife_lib.h"
#include<memory>

class LIFELIB_API Driver
{
public:
    Driver(char type_of_engine, char type_of_viewer, int width, int height);
    void set_delay(int delay);
    void set_init_conditions(std::vector<bool>& input);
    void set_board_type(char type);
    int start();
    bool is_over(const Board& new_board) const;
    bool next();

private:
    int _delay = 1000;
    std::vector<bool> _input;
    Board _board;
    std::unique_ptr<IEngine> _engine;
    std::unique_ptr<IViewer> _viewer;
};

#endif
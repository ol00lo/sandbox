#include "driver.hpp"
#include "cmd_viewer.hpp"
#include "life_engine.hpp"
#include <thread>

Driver::Driver(char type_of_engine, char type_of_viewer, int height, int width) : _board(height, width)
{

    if (type_of_viewer == 'c')
    {
        _viewer = std::make_unique<CmdViewer>();
    }
    else
    {
        throw std::runtime_error("This type of viewer is not supported");
    }
    if (type_of_engine == 'c')
    {
        _engine = std::make_unique<LifeEngine>();
    }
    else
    {
        throw std::runtime_error("This type of engine is not supported");
    }
}

void Driver::set_delay(int delay)
{
    _delay = delay;
}

void Driver::set_init_conditions(std::vector<bool>& input)
{
    if (input.size() != _board.ncols() * _board.nrows())
    {
        throw std::runtime_error("Incorrect input");
    }
    _input = std::move(input);
}

int Driver::start()
{
    int steps = 0;
    _board.add_data(_input);
    _viewer->display(_board);
    while (next())
    {
        steps++;
        _viewer->display(_board);
        std::this_thread::sleep_for(std::chrono::milliseconds(_delay));
    }
    _viewer->game_over();
    return steps;
}

bool Driver::next()
{
    Board new_board = _engine->step(_board);
    if (is_over(new_board))
    {
        return false;
    }
    else
    {
        std::swap(_board, new_board);
        return true;
    }
}

bool Driver::is_over(const Board& new_board) const
{
    bool diff = (new_board == _board);
    bool life = new_board.is_alldead();

    if (diff || !life)
        return true;
    else
        return false;
}

void Driver::set_board_type(char type)
{
    _board.set_type(type);
}
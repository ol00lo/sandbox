#include "driver.hpp"
#include "cmd_viewer.hpp"
#include "life_engine.hpp"
#include <thread>

Driver::Driver(const Arguments& a) : _arg(a), _board(_arg.height, _arg.width, _arg.type_board)
{
    if (_arg.viewer_type == 'c')
    {
        _viewer = std::make_unique<CmdViewer>();
    }
    else
    {
        throw std::runtime_error("This type of viewer is not supported");
    }
    if (_arg.engine_type == 'c')
    {
        _engine = std::make_unique<LifeEngine>();
    }
    else
    {
        throw std::runtime_error("This type of engine is not supported");
    }
    _arg.validate();
    _board.add_data(std::move(_arg.input));
}

void Driver::start()
{
    _viewer->display(_board);
    while (next())
    {
        _viewer->display(_board);
        std::this_thread::sleep_for(std::chrono::milliseconds(_arg.delay));
    }
    _viewer->game_over();
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

#include "driver.hpp"
#include "cmd_viewer.hpp"
#include "reg_engine.hpp"

Driver::Driver(const Arguments& a) : arg(a), board(arg.height, arg.width, arg.type_board)
{
    if (arg.viewer_type == 'c')
    {
        viewer = std::make_unique<cmd_viewer>();
    }
    else
    {
        throw std::runtime_error("This type of viewer doesn't support");
    }
    if (arg.engine_type == 'c')
    {
        engine = std::make_unique<reg_engine>();
    }
    else
    {
        throw std::runtime_error("This type of engine doesn't support");
    }
    arg.validate();
    board.add_data(std::move(arg.input));
}

void Driver::start()
{
    viewer->display(board);
    while (next())
    {
        viewer->display(board);
        std::this_thread::sleep_for(std::chrono::milliseconds(arg.delay));
    }
    viewer->game_over();
}

bool Driver::next()
{
    Board new_board = engine->step(board);
    if (is_over(new_board))
    {
        return false;
    }
    else
    {
        board = new_board;
        return true;
    }
}

bool Driver::is_over(const Board& new_board) const
{
    bool diff = (new_board == board);
    bool life = new_board.is_alldead();

    if (diff || !life)
        return true;
    else
        return false;
}

#include "game.hpp"

Board reg_engine::step(const Board& board) const
{
    Board new_board(board);
    for (int i = 0; i < board.nrows(); i++)
    {
        for (int j = 0; j < board.ncols(); j++)
        {
            int nneigh = count_of_neighbors(board, i, j);
            if (board.at(i, j) && nneigh != 2 && nneigh != 3)
            {
                new_board.set_at(i, j, false);
            }
            else if (!board.at(i, j) && nneigh == 3)
            {
                new_board.set_at(i, j, true);
            }
        }
    }
    return new_board;
}

int reg_engine::count_of_neighbors(const Board& board, int row, int col) const
{
    int res = 0;
    if (board.at(row, col - 1))
        res++;
    if (board.at(row - 1, col))
        res++;
    if (board.at(row, col + 1))
        res++;
    if (board.at(row + 1, col))
        res++;
    if (board.at(row - 1, col - 1))
        res++;
    if (board.at(row + 1, col - 1))
        res++;
    if (board.at(row - 1, col + 1))
        res++;
    if (board.at(row + 1, col + 1))
        res++;
    return res;
}

void cmd_viewer::display(const Board& board) 
{
    _ncols = board.ncols();
    _nrows = board.nrows();
    std::cout << "\033[2J\033[H";

    std::cout << " ";
    for (int j = 0; j <= board.ncols(); ++j)
    {
        std::cout << "_";
    }
    std::cout << "\n";

    for (int i = 0; i < board.nrows(); ++i)
    {
        std::cout << "|";
        for (int j = 0; j < board.ncols(); ++j)
        {
            if (is_alive(board, i, j))
            {
                std::cout << "#";
            }
            else
            {
                std::cout << " ";
            }
        }
        std::cout << " |\n";
    }

    std::cout << " ";
    for (int j = 0; j <= board.ncols(); ++j)
    {
        std::cout << "-";
    }
    std::cout << "\n";
}

void cmd_viewer::game_over() 
{
    std::cout << "\033[H";
    std::cout << " ";
    for (int j = 0; j <= _ncols; ++j)
    {
        std::cout << "_";
    }
    std::cout << "\n";
    for (int i = 0; i < _nrows; ++i)
    {
        std::cout << "|";
        if (i == _nrows / 2)
        {
            std::cout << std::setw(_ncols) << "ALLDEAD";
        }
        else
        {
            for (int j = 0; j < _ncols; ++j)
            {
                std::cout << " ";
            }
        }
        std::cout << " |\n";
    }
    std::cout << " ";
    for (int j = 0; j <= _ncols; ++j)
    {
        std::cout << "-";
    }
    std::cout << "\n";
}

bool cmd_viewer::is_alive(const Board& board, int row, int col) const
{
    if (row >= board.nrows() || row < 0 || col >= board.ncols() || col < 0)
    {
        throw std::runtime_error("out of range");
    }
    return board.at(row, col);
}

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

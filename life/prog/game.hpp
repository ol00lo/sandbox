#ifndef GAME_HPP
#define GAME_HPP
#include "arguments.hpp"
#include "board.hpp"
#include <chrono>
#include <iomanip>
#include <random>
#include <thread>

class IEngine
{
public:
    virtual ~IEngine() = default;
    virtual Board step(const Board& board) const = 0;
};

class IViewer
{
public:
    virtual ~IViewer() = default;
    virtual void display(const Board& board) = 0;
    virtual void game_over() = 0;
};

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

class reg_engine : public IEngine
{
public:
    Board step(const Board& board) const override;

private:
    int count_of_neighbors(const Board& board, int row, int col) const;
};


class cmd_viewer : public IViewer
{
public:
    void display(const Board& board) override;
    void game_over() override;

private:
    bool is_alive(const Board& board, int row, int col) const;
    int _ncols = 0;
    int _nrows = 0;
};

#endif // !GAME_HPP
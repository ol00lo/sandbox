#ifndef REG_ENGINE_HPP
#define REG_ENGINE_HPP
#include "i_engine.hpp"

class LifeEngine : public IEngine
{
public:
    Board step(const Board& board) const override;

private:
    int count_of_neighbors(const Board& board, int row, int col) const;
};
#endif
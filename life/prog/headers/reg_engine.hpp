#ifndef REG_ENGINE_HPP
#define REG_ENGINE_HPP
#include "engine.hpp"

class reg_engine : public IEngine
{
public:
    Board step(const Board& board) const override;

private:
    int count_of_neighbors(const Board& board, int row, int col) const;
};
#endif
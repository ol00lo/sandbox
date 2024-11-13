#ifndef ENGINE_HPP
#define ENGINE_HPP
#include "board.hpp"

class LIFELIB_API IEngine
{
public:
    virtual ~IEngine() = default;
    virtual Board step(const Board& board) const = 0;
};
#endif
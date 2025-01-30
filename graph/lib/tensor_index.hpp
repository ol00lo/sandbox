#ifndef TENSOR_INDEX_HPP
#define TENSOR_INDEX_HPP

#include <array>

namespace g
{
class Shape : public std::array<int, 4>
{
public:
    Shape(const std::array<int, 4>& shape) : std::array<int, 4>{shape} {}
    Shape(int b, int i, int j, int c) : std::array<int, 4>{b, i, j, c}{}
    Shape(int i, int j, int c): std::array<int, 4>{1, i, j, c}{}
    Shape(int j, int c): std::array<int, 4>{1, 1, j, c}{}
    Shape(int c) : std::array<int, 4>{1, 1, 1, c}{}
};

class Index : public Shape
{
public:
    Index(int c) : Shape(0,0,0,c) {}
    Index(int j, int c): Shape(0,0,j,c) {}
    Index(int i, int j, int c): Shape(0,i,j,c) {}
    Index(int i, int j, int c, int b) : Shape(i,j,c,b) {}
    Index(int linear, const Shape& shape);
    int to_linear(const Shape& shape) const;
};
} // namespace g
#endif
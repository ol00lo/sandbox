#ifndef TENSOR_INDEX_HPP
#define TENSOR_INDEX_HPP

#include <array>

namespace g
{
using arr4 = std::array<int, 4>;

class Shape : public arr4
{
public:
    Shape(const arr4& shape) : arr4{shape}{}
    Shape(int b, int i, int j, int c) : arr4{b, i, j, c}{}
    Shape(int i, int j, int c) : arr4{1, i, j, c}{}
    Shape(int j, int c) : arr4{1, 1, j, c}{}
    Shape(int c) : arr4{1, 1, 1, c}{}
    int n_indexes() const;
};

class Index : public arr4
{
public:
    Index(int c) : arr4{0, 0, 0, c}{}
    Index(int j, int c) : arr4{0, 0, j, c}{}
    Index(int i, int j, int c) : arr4{0, i, j, c}{}
    Index(int i, int j, int c, int b) : arr4{i, j, c, b}{}
    Index(int linear, const Shape& shape);
    int to_linear(const Shape& shape) const;
};
} // namespace g
#endif
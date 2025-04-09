#ifndef TENSOR_INDEX_HPP
#define TENSOR_INDEX_HPP

#include <array>

namespace g
{
using Arr4 = std::array<int, 4>;

class Shape : public Arr4
{
public:
    Shape(const Arr4& shape) : Arr4{shape} {};
    Shape(int b, int i, int j, int c) : Arr4{b, i, j, c} {};
    Shape(int i, int j, int c) : Arr4{1, i, j, c} {};
    Shape(int j, int c) : Arr4{1, 1, j, c} {};
    Shape(int c) : Arr4{1, 1, 1, c} {};
    Shape(const std::string s);
    int n_indexes() const;
    std::string to_string();
};

class Index : public Arr4
{
public:
    Index(int c) : Arr4{0, 0, 0, c} {};
    Index(int j, int c) : Arr4{0, 0, j, c} {};
    Index(int i, int j, int c) : Arr4{0, i, j, c} {};
    Index(int i, int j, int c, int b) : Arr4{i, j, c, b} {};
    Index(int linear, const Shape& shape);
    int to_linear(const Shape& shape) const;
};
} // namespace g
#endif
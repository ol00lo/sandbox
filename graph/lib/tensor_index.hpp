#ifndef TENSOR_INDEX_HPP
#define TENSOR_INDEX_HPP

#include <array>

namespace g
{
class TensorIndex : public std::array<int, 4>
{
public:
    TensorIndex(int c);
    TensorIndex(int j, int c);
    TensorIndex(int i, int j, int c);
    TensorIndex(int linear, const TensorIndex& shape);
    int to_linear(const TensorIndex& shape) const;
};
} // namespace g
#endif
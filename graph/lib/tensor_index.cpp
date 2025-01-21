#include "tensor_index.hpp"
#include <iostream>

using namespace g;

TensorIndex::TensorIndex(int c)
{
    (*this)[0] = 1;
    (*this)[1] = 1;
    (*this)[2] = 1;
    (*this)[3] = c;
}
TensorIndex::TensorIndex(int j, int c)
{
    (*this)[0] = 1;
    (*this)[1] = 1;
    (*this)[2] = j;
    (*this)[3] = c;
}

TensorIndex::TensorIndex(int i, int j, int c)
{
    (*this)[0] = 1;
    (*this)[1] = i;
    (*this)[2] = j;
    (*this)[3] = c;
}

TensorIndex::TensorIndex(int linear, const TensorIndex& shape)
{
    if (linear < 0 || linear >= to_linear(shape))
    {
        throw std::runtime_error("out of range.");
    }

    (*this)[3] = linear % shape[3];
    linear /= shape[3];
    (*this)[2] = linear % shape[2];
    linear /= shape[2];
    (*this)[1] = linear % shape[1];
    linear /= shape[1];
    (*this)[0] = linear;
}

int TensorIndex::to_linear(const TensorIndex& shape) const
{
    if (shape[0] > 0 && shape[1] > 0 && shape[2] > 0 && shape[3] > 0)
    {
        return (*this)[0] * shape[1] * shape[2] * shape[3] + (*this)[1] * shape[2] * shape[3] + (*this)[2] * shape[3] +
               (*this)[3];
    }
    else
    {
        throw std::runtime_error("Invalid shape.");
    }
}

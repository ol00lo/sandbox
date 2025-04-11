#include "tensor_index.hpp"
#include <iostream>
#include <sstream>

using namespace g;

Shape::Shape(const std::string s)
{
    std::stringstream ss(s);
    char ch;
    double num;
    for (int i = 0; i < 4; ++i)
    {
        ss >> ch;
        ss >> num;
        (*this)[i] = int(num);
    }
}

int Shape::n_indexes() const
{
    return (*this)[0] * (*this)[1] * (*this)[2] * (*this)[3];
}

Index::Index(int linear, const Shape& shape)
{
    if (linear < 0 || linear >= shape.n_indexes())
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

int Index::to_linear(const Shape& shape) const
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

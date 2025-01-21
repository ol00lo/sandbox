#include "tensor.hpp"

using namespace g;

void Tensor::clear_data()
{
    for (int i = 0; i < _data.size(); i++)
    {
        _data[i] = 0;
    }
}
double Tensor::operator[](TensorIndex ind) const
{
    int i = _shape.to_linear(ind);
    return _data[i];
}
void Tensor::add(const Tensor& other)
{
    for (int i = 0; i < _data.size(); i++)
    {
        _data[i] += other._data[i];
    }
}
void Tensor::sub(const Tensor& other)
{
    for (int i = 0; i < _data.size(); i++)
    {
        _data[i] -= other._data[i];
    }
}
void Tensor::mult(const Tensor& other)
{
    for (int i = 0; i < _data.size(); i++)
    {
        _data[i] *= other._data[i];
    }
}
void Tensor::div(const Tensor& other)
{
    for (int i = 0; i < _data.size(); i++)
    {
        if (other._data[i] == 0)
            throw std::runtime_error("Division by zero");
        _data[i] /= other._data[i];
    }
}
Tensor& Tensor::operator=(const Tensor& t)
{
    _data = t._data;
    _shape = t._shape;
    return *this;
}

Tensor::operator std::vector<double>() const
{
    return _data;
}

Tensor g::add(const Tensor& t1, const Tensor& t2)
{
    Tensor res(t1);
    res.add(t2);
    return res;
}
Tensor g::mult(const Tensor& t1, const Tensor& t2)
{
    Tensor res(t1);
    res.mult(t2);
    return res;
}
Tensor g::sub(const Tensor& t1, const Tensor& t2)
{
    Tensor res(t1);
    res.sub(t2);
    return res;
}
Tensor g::div(const Tensor& t1, const Tensor& t2)
{
    Tensor res(t1);
    res.div(t2);
    return res;
}
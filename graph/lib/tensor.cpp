#include "tensor.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>

using namespace g;

Tensor::Tensor(Shape shape, const std::vector<double>& data) : _shape(shape), _data(data)
{
    if (data.size() != shape[0] * shape[1] * shape[2] * shape[3])
    {
        throw std::runtime_error("Incorrect tensor shape.");
    }
}
Tensor::Tensor(Shape shape, std::vector<double>&& data) : _shape(shape), _data(std::move(data))
{
    if (_data.size() != shape[0] * shape[1] * shape[2] * shape[3])
    {
        throw std::runtime_error("Incorrect tensor shape.");
    }
}
Tensor::Tensor(Shape shape, double a) : _shape(shape)
{
    _data = std::vector<double>(shape[0] * shape[1] * shape[2] * shape[3], a);
}

void Tensor::set_zero()
{
    std::fill(_data.begin(), _data.end(), 0);
}
double Tensor::operator[](Index ind) const
{
    return _data[ind.to_linear(_shape)];
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
    if (std::any_of(other._data.begin(), other._data.end(), [](double d) { return d == 0; }))
    {
        throw std::runtime_error("Division by zero");
    }
    for (int i = 0; i < _data.size(); i++)
    {
        _data[i] /= other._data[i];
    }
}

void Tensor::scalar_mult(double a)
{
    for (int i = 0; i < _data.size(); i++)
    {
        _data[i] *= a;
    }
}
Tensor& Tensor::operator=(const Tensor& t)
{
    _data = t._data;
    _shape = t._shape;
    return *this;
}
bool Tensor::operator==(const Tensor& other) const
{
    if (_shape != other._shape)
    {
        return false;
    }

    return _data == other._data;
}
Shape Tensor::get_shape() const
{
    return _shape;
}
void Tensor::serialize(nlohmann::json& js) const
{
    js["shape"] = _shape;
    js["value"] = _data;
}
void Tensor::to_string(std::ostream& os) const
{
    for (int b = 0; b < _shape[0]; b++)
    {
        os << "[";
        for (int i = 0; i < _shape[1]; i++)
        {
            os << "[";
            for (int j = 0; j < _shape[2]; j++)
            {
                os << "[";
                for (int c = 0; c < _shape[3]; c++)
                {
                    Index ind(b, i, j, c);
                    double d = _data[ind.to_linear(_shape)];
                    os << std::fixed << std::setprecision(2) << d << "  ";
                }
                os << "]";
            }
            os << "]";
        }
        os << "]";
    }
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
Tensor g::scalar_mult(double a, const Tensor& t)
{
    Tensor res(t);
    res.scalar_mult(a);
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
#include "tensor.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cmath>

using namespace g;

Tensor::Tensor(Shape shape, const std::vector<double>& data) : _shape(shape), _data(data)
{
    if (data.size() != shape.n_indexes())
    {
        throw std::runtime_error("Incorrect tensor shape.");
    }
}
Tensor::Tensor(Shape shape, double a) : _shape(shape)
{
    _data = std::vector<double>(shape.n_indexes(), a);
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
void Tensor::sin()
{
    for (int i = 0; i < _data.size(); i++)
    {
        _data[i] = std::sin(_data[i]);
    }
}
void Tensor::cos()
{
    for (int i = 0; i < _data.size(); i++)
    {
        _data[i] = std::cos(_data[i]);
    }
}
void Tensor::tg()
{
    for (int i = 0; i < _data.size(); i++)
    {
        _data[i] = std::tan(_data[i]);
    }
}
void Tensor::ctg()
{
    for (int i = 0; i < _data.size(); i++)
    {
        double t = std::tan(_data[i]);
        if (t != 0)
        {
            _data[i] = 1 / t;
        }
        else
        {
            throw std::runtime_error("Division by zero");
        }
    }
}

void Tensor::sqrt()
{
    for (int i = 0; i < _data.size(); i++)
    {
        _data[i] *= _data[i];
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
void Tensor::write(std::ostream& os) const
{
    os << "data = \n";
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
                    os << std::fixed << std::setprecision(4) << d << "  ";
                }
                os << "]";
                if (j != _shape[2] - 1)
                    os << "\n";
            }
            os << "]";
            if (i != _shape[1] - 1)
                os << "\n";
        }
        os << "]";
        if (b != _shape[0] - 1)
            os << "\n";
    }
    os << ",  shape = (" << _shape[0] << ", " << _shape[1] << ", " << _shape[2] << ", " << _shape[3] << ")" << std::endl;
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

Tensor g::sin(const Tensor& t)
{
    Tensor res(t);
    res.sin();
    return res;
}
Tensor g::cos(const Tensor& t)
{
    Tensor res(t);
    res.cos();
    return res;
}
Tensor g::tg(const Tensor& t)
{
    Tensor res(t);
    res.tg();
    return res;
}
Tensor g::ctg(const Tensor& t)
{
    Tensor res(t);
    res.ctg();
    return res;
}
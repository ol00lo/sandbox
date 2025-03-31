#include "tensor.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace g;

Tensor::Tensor(Shape shape, const std::vector<double>& data) : shape_(shape), data_(data)
{
    if (data.size() != shape.n_indexes())
    {
        throw std::runtime_error("Incorrect tensor shape.");
    }
}
Tensor::Tensor(Shape shape, double a) : shape_(shape)
{
    data_ = std::vector<double>(shape.n_indexes(), a);
}

void Tensor::set_zero()
{
    std::fill(data_.begin(), data_.end(), 0);
}
double Tensor::operator[](Index ind) const
{
    return data_[ind.to_linear(shape_)];
}
void Tensor::add(const Tensor& other)
{
    for (int i = 0; i < data_.size(); i++)
    {
        data_[i] += other.data_[i];
    }
}
void Tensor::sub(const Tensor& other)
{
    for (int i = 0; i < data_.size(); i++)
    {
        data_[i] -= other.data_[i];
    }
}
void Tensor::mult(const Tensor& other)
{
    for (int i = 0; i < data_.size(); i++)
    {
        data_[i] *= other.data_[i];
    }
}
void Tensor::div(const Tensor& other)
{
    if (std::any_of(other.data_.begin(), other.data_.end(), [](double d) { return d == 0; }))
    {
        throw std::runtime_error("Division by zero");
    }
    for (int i = 0; i < data_.size(); i++)
    {
        data_[i] /= other.data_[i];
    }
}

void Tensor::scalar_mult(double a)
{
    for (int i = 0; i < data_.size(); i++)
    {
        data_[i] *= a;
    }
}
Tensor& Tensor::operator=(const Tensor& t)
{
    data_ = t.data_;
    shape_ = t.shape_;
    return *this;
}
bool Tensor::operator==(const Tensor& other) const
{
    if (shape_ != other.shape_)
    {
        return false;
    }

    return data_ == other.data_;
}
Shape Tensor::get_shape() const
{
    return shape_;
}
const std::vector<double>& Tensor::data() const
{
    return data_;
}

void Tensor::write(std::ostream& os) const
{
    os << "data = \n";
    for (int b = 0; b < shape_[0]; b++)
    {
        os << "[";
        for (int i = 0; i < shape_[1]; i++)
        {
            os << "[";
            for (int j = 0; j < shape_[2]; j++)
            {
                os << "[";
                for (int c = 0; c < shape_[3]; c++)
                {
                    Index ind(b, i, j, c);
                    double d = data_[ind.to_linear(shape_)];
                    os << std::fixed << std::setprecision(4) << d << "  ";
                }
                os << "]";
                if (j != shape_[2] - 1)
                    os << "\n";
            }
            os << "]";
            if (i != shape_[1] - 1)
                os << "\n";
        }
        os << "]";
        if (b != shape_[0] - 1)
            os << "\n";
    }
    os << ",  shape = (" << shape_[0] << ", " << shape_[1] << ", " << shape_[2] << ", " << shape_[3] << ")" << std::endl;
}

Tensor g::add(const Tensor& t1, const Tensor& t2)
{
    if (t1.get_shape() != t2.get_shape())
    {
        throw std::runtime_error("Incorrect shapes");
    }
    Tensor res(t1);
    res.add(t2);
    return res;
}
Tensor g::mult(const Tensor& t1, const Tensor& t2)
{
    if (t1.get_shape() != t2.get_shape())
    {
        throw std::runtime_error("Incorrect shapes");
    }
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
    if (t1.get_shape() != t2.get_shape())
    {
        throw std::runtime_error("Incorrect shapes");
    }
    Tensor res(t1);
    res.sub(t2);
    return res;
}
Tensor g::div(const Tensor& t1, const Tensor& t2)
{
    if (t1.get_shape() != t2.get_shape())
    {
        throw std::runtime_error("Incorrect shapes");
    }
    Tensor res(t1);
    res.div(t2);
    return res;
}

Tensor g::sin(const Tensor& t)
{
    Tensor res(t);
    res.apply_oper([](double x) { return std::sin(x); });
    return res;
}
Tensor g::cos(const Tensor& t)
{
    Tensor res(t);
    res.apply_oper([](double x) { return std::cos(x); });
    return res;
}
Tensor g::tg(const Tensor& t)
{
    Tensor res(t);
    res.apply_oper([](double x) { return std::tan(x); });
    return res;
}
Tensor g::ctg(const Tensor& t)
{
    Tensor res(t);
    res.apply_oper([](double x) {
        auto tg = std::tan(x);
        if (tg == 0) throw std::runtime_error("Division by zero");
        return 1 / tg; });
    return res;
}
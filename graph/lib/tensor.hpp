#ifndef TENSOR_HPP
#define TENSOR_HPP

#include "graph.hpp"
#include "tensor_index.hpp"
#include <array>
namespace g
{
class Tensor
{
public:
    Tensor(Tensor& t) : _data(t._data), _shape(t._shape)
    {
    }
    Tensor(std::vector<double> data, TensorIndex shape) : _data(data), _shape(shape)
    {
    }
    Tensor(std::vector<double> data) : _data(data), _shape(data.size())
    {
    }
    void clear_data();
    double operator[](TensorIndex ind) const;
    void add(const Tensor& other);
    void sub(const Tensor& other);
    void mult(const Tensor& other);
    void div(const Tensor& other);
    Tensor& operator=(const Tensor& t);
    operator std::vector<double>() const;
    friend std::ostream& operator<<(std::ostream& os, const Tensor& tensor)
    {
        for (int i = 0; i < tensor._data.size(); i++)
        {
            os << tensor._data[i] << " ";
        }
        return os;
    }

private:
    std::vector<double> _data;
    TensorIndex _shape;
};
Tensor add(const Tensor& t1, const Tensor& t2);
Tensor mult(const Tensor& t1, const Tensor& t2);
Tensor sub(const Tensor& t1, const Tensor& t2);
Tensor div(const Tensor& t1, const Tensor& t2);
} // namespace g

#endif
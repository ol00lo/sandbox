#ifndef TENSOR_HPP
#define TENSOR_HPP

#include "graph.hpp"
#include "tensor_index.hpp"
#include <nlohmann/json.hpp>
#include <array>
namespace g
{
class Tensor
{
public:
    Tensor(const Tensor& t) : _data(t._data), _shape(t._shape) {}

    Tensor(const std::vector<double>& data) : _data(data), _shape(data.size()) {}
    Tensor(std::vector<double>&& data) : _data(std::move(data)), _shape(_data.size()) {}
    Tensor(Shape shape, const std::vector<double>& data);
    Tensor(Shape shape, std::vector<double>&& data);
    Tensor(Shape shape, double a);
    void set_zero();
    double operator[](Index ind) const;
    void add(const Tensor& other);
    void sub(const Tensor& other);
    void mult(const Tensor& other);
    void div(const Tensor& other);
    void scalar_mult(double a);
    Shape get_shape() const;
    Tensor& operator=(const Tensor& t);
    bool operator==(const Tensor& other) const;
    void serialize(nlohmann::json& js) const;
    void to_string(std::ostream& os) const;

private:
    std::vector<double> _data;
    Shape _shape;
};
Tensor add(const Tensor& t1, const Tensor& t2);
Tensor mult(const Tensor& t1, const Tensor& t2);
Tensor scalar_mult(double a, const Tensor& t);
Tensor sub(const Tensor& t1, const Tensor& t2);
Tensor div(const Tensor& t1, const Tensor& t2);
static std::ostream& operator<<(std::ostream& os, const Tensor& tensor)
{
    tensor.to_string(os);
    return os;
}
} // namespace g

#endif
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
    Tensor(Tensor&& t): _data(std::move(t._data)), _shape(std::move(t._shape)){}

    Tensor(const std::vector<double>& data) : _data(data), _shape(data.size()) {}
    Tensor(Shape shape, const std::vector<double>& data);
    Tensor(Shape shape, double a);
    void set_zero();
    double operator[](Index ind) const;
    void add(const Tensor& other);
    void sub(const Tensor& other);
    void mult(const Tensor& other);
    void div(const Tensor& other);
    void sin();
    void cos();
    void tg();
    void ctg();
    void sqrt();
    void scalar_mult(double a);
    Shape get_shape() const;
    Tensor& operator=(const Tensor& t);
    bool operator==(const Tensor& other) const;
    void serialize(nlohmann::json& js) const;
    void write(std::ostream& os = std::cout) const;
    friend std::ostream& operator<<(std::ostream& os, const Tensor& tensor)
    {
        tensor.write(os);
        return os;
    }

private:
    std::vector<double> _data;
    Shape _shape;
};
Tensor add(const Tensor& t1, const Tensor& t2);
Tensor mult(const Tensor& t1, const Tensor& t2);
Tensor scalar_mult(double a, const Tensor& t);
Tensor sub(const Tensor& t1, const Tensor& t2);
Tensor div(const Tensor& t1, const Tensor& t2);
Tensor sin(const Tensor& t);
Tensor cos(const Tensor& t1);
Tensor tg(const Tensor& t1);
Tensor ctg(const Tensor& t1);
} // namespace g

#endif
#ifndef TENSOR_HPP
#define TENSOR_HPP

#include "graph.hpp"
#include "tensor_index.hpp"
#include <array>
#include <nlohmann/json.hpp>

namespace g
{
class Tensor
{
public:
    Tensor() : _shape(1), _data(0) {};
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
    void scalar_mult(double a);
    Shape get_shape() const;
    const std::vector<double>& data() const;
    Tensor& operator=(const Tensor& t);
    bool operator==(const Tensor& other) const;
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
} // namespace g

namespace nlohmann
{
template <>
struct adl_serializer<g::Tensor>
{
    static void to_json(json& j, const g::Tensor& t)
    {
        j = json{{"value", t.data()}, {"shape", t.get_shape()}};
    }

    static void from_json(const json& j, g::Tensor& t)
    {
        std::vector<double> data = j.at("value").get<std::vector<double>>();
        g::Shape shape = j.at("shape").get<g::Arr4>();
        t = g::Tensor(shape, data);
    }
};
} // namespace nlohmann
#endif
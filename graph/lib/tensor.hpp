#ifndef TENSOR_HPP
#define TENSOR_HPP

#include "graph.hpp"
#include "tensor_index.hpp"
#include <array>
#include <nlohmann/json.hpp>
#include <vector>

namespace g
{
class Tensor
{
public:
    Tensor() : shape_(1), data_(0){};
    Tensor(const Tensor& t) : data_(t.data_), shape_(t.shape_){};
    Tensor(Tensor&& t) : data_(std::move(t.data_)), shape_(std::move(t.shape_)){};
    Tensor(const std::vector<double>& data) : data_(data), shape_(data.size()){};
    Tensor(Shape shape, const std::vector<double>& data);
    Tensor(Shape shape, double a);

    void set_zero();
    double operator[](Index ind) const;
    Shape shape() const;
    const std::vector<double>& data() const;

    void add(const Tensor& other);
    void sub(const Tensor& other);
    void mult(const Tensor& other);
    void div(const Tensor& other);
    void scalar_mult(double a);

    Tensor& operator=(const Tensor& t);
    bool operator==(const Tensor& other) const;
    void write(std::ostream& os = std::cout) const;
    friend std::ostream& operator<<(std::ostream& os, const Tensor& tensor)
    {
        tensor.write(os);
        return os;
    }
    template <class Oper>
    void apply_oper(Oper&& op)
    {
        for (auto& x : data_)
        {
            x = op(x);
        }
    }

private:
    std::vector<double> data_;
    Shape shape_;
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

namespace nlohmann
{
template <>
struct adl_serializer<g::Tensor>
{
    static void to_json(json& j, const g::Tensor& t)
    {
        j = json{{"value", t.data()}, {"shape", json(t.shape())}};
    }

    static void from_json(const json& j, g::Tensor& t)
    {
        std::vector<double> data = j.at("value").get<std::vector<double>>();
        g::Shape shape = j.at("shape").get<std::string>();
        t = g::Tensor(shape, data);
    }
};
} // namespace nlohmann
#endif

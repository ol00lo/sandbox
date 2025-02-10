#ifndef POINT_HPP
#define POINT_HPP
#include <array>
#include <sstream>
#include <string>

template <typename U, std::size_t N>
constexpr static auto trimArray(const std::array<U, N>& arr)
{
    std::array<U, N - 1> newArr{};
    for (std::size_t i = 1; i < N; ++i)
    {
        newArr[i - 1] = arr[i];
    }
    return newArr;
}

template <class T, int Dim>
struct Point;

template <class T, int Dim>
struct PointImpl
{
    T _value;

    PointImpl(T val) : _value(val) {}

    virtual std::string to_help_string() = 0;
    virtual T measure(const Point<T, Dim>& p2) = 0;

    T distance(const Point<T, Dim>& p2)
    {
        return std::sqrt(measure(p2));
    }

    std::string to_string()
    {
        return "( " + to_help_string() + " )";
    }
};

template <class T, int Dim>
struct Point : public PointImpl<T, Dim>
{
    static_assert(Dim > 0 && Dim <= 3, "Dim should be 1, 2 or 3");
    Point<T, Dim - 1> pred;
    Point(std::array<T, Dim> arr) : PointImpl<T, Dim>(arr[0]), pred(trimArray(arr)){}

    T measure(const Point<T, Dim>& p2) override
    {
        T this_dim_dist = _value - p2._value;
        T pred_dim_measure = pred.measure( p2.pred);
        return this_dim_dist * this_dim_dist + pred_dim_measure;
    }
    std::string to_help_string() override
    {
        return std::to_string(_value) + ", " + pred.to_help_string();
    }

    constexpr static Point<T, Dim> from_string_impl(const std::string& inp)
    {
        std::string cl_inp;
        for (char c : inp)
        {
            if (c != ' ' && c != '(' && c != ')')
                cl_inp += c;
        }

        std::stringstream ss(cl_inp);
        std::string val;
        std::array<T, Dim> values;
        std::size_t index = 0;

        while (std::getline(ss, val, ','))
        {
            std::stringstream valueStream(val);
            valueStream >> values[index++];
        }
        if (values.size() != Dim)
        {
            throw std::runtime_error("Incorrect input");
        }
        return Point<T, Dim>(values);
    }
};

template <class T>
struct Point<T, 1> : public PointImpl<T, 1>
{
    Point(std::array<T, 1> arr): PointImpl<T, 1>(arr[0]){}
    T measure(const Point<T, 1>& p2) override 
    {
        T dist = _value - p2._value;
        return dist * dist;
    }
    std::string to_help_string() override
    {
        return std::to_string(_value) + ")";
    }
};
#endif
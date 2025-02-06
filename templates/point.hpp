#ifndef POINT_HPP
#define POINT_HPP
#include <array>
#include <string>
#include <sstream>

template <class T, int Dim>
struct Point
{
    static_assert(Dim > 0 && Dim <= 3, "Dim should be 1, 2 or 3");
    Point<T, Dim - 1> pred;
    T value;
    Point(std::array<T, Dim> arr) : pred(Point<T, Dim - 1>(trimArray(arr))), value(arr[0])
    {
    }

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
    static T distance(Point<T, Dim> p1, Point<T, Dim> p2)
    {
        T this_dim_dist = p1.value - p2.value;
        T pred_dim_dist = Point<T, Dim - 1>::distance(p1.pred, p2.pred);
        return std::sqrt(this_dim_dist * this_dim_dist + pred_dim_dist * pred_dim_dist);
    }
    std::string to_string()
    {
        return "( " + to_help_string();
    }
    std::string to_help_string()
    {
        return std::to_string(value) + ", " + pred.to_help_string();
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
struct Point<T, 1>
{
    T value;
    Point(std::array<T, 1> arr)
    {
        value = arr[0];
    }
    static T distance(Point<T, 1> p1, Point<T, 1> p2)
    {
        T dist = p1.value - p2.value;
        return std::sqrt(dist * dist);
    }
    std::string to_help_string()
    {
        return std::to_string(value) + ")";
    }
    std::string to_string()
    {
        return "(" + to_help_string();
    }
};
#endif
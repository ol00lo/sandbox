#ifndef HUNDREDS_HPP
#define HUNDREDS_HPP
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>

namespace
{
std::string concatenate_with_delimiter(const std::string& delimiter, const std::initializer_list<std::string>& inp)
{
    std::ostringstream res;
    bool first = true;
    for (const auto& str : inp)
    {
        if (!str.empty())
        {
            if (!first)
            {
                res << delimiter;
            }
            res << str;
            first = false;
        }
    }
    return res.str();
}
} // namespace

namespace num
{
namespace impl
{
template <int... D>
constexpr bool all_zero()
{
    return (... && (D == 0));
}

template <int... D>
constexpr bool is_in_range()
{
    return (... && (D >= 0 && D < 10));
}

template <typename T, typename = void>
struct has_digit_name : std::false_type{};

template <typename T>
struct has_digit_name<T, std::void_t<decltype(T::name)>> : std::true_type{};

template <int T, int U, bool is_thousands = false>
std::string suffix()
{
    if constexpr (is_thousands)
    {
        if constexpr (U == 1 && T != 1)
        {
            return "а";
        }
        else if constexpr ((U == 2 || U == 3 || U == 4) && T != 1)
        {
            return "и";
        }
        else
        {
            return "";
        }
    }
    else if constexpr (U == 1 && T != 1)
    {
        return "";
    }
    else if constexpr ((U == 2 || U == 3 || U == 4) && T != 1)
    {
        return "а";
    }
    else
    {
        return "ов";
    }
}
template <int D>
struct Units
{
    static std::string unit()
    {
        if constexpr (D == 0)
            return "";
        else if constexpr (D == 1)
            return "один";
        else if constexpr (D == 2)
            return "два";
        else if constexpr (D == 3)
            return "три";
        else if constexpr (D == 4)
            return "четыре";
        else if constexpr (D == 5)
            return "пять";
        else if constexpr (D == 6)
            return "шесть";
        else if constexpr (D == 7)
            return "семь";
        else if constexpr (D == 8)
            return "восемь";
        else if constexpr (D == 9)
            return "девять";
    }

    static std::string from_hundreds()
    {
        if constexpr (D == 2 || D == 1 || D == 0)
            return "";
        else
            return unit();
    }
    static std::string from_thousands()
    {
        if constexpr (D == 1)
            return "одна";
        else if constexpr (D == 2)
            return "две";
        else
            return unit();
    }
    static std::string apply()
    {
        return unit();
    }
};

template <int C, int D, bool from_thousands = false>
struct Tens
{
    static constexpr std::string name()
    {
        if constexpr (C == 2 || C == 3)
        {
            return "дцать";
        }
        else
        {
            return "десят";
        }
    }
    static std::string ten()
    {
        std::string unit = Units<D>::unit();
        if constexpr (from_thousands)
        {
            unit = Units<D>::from_thousands();
        }
        if constexpr (C == 0)
            return unit;
        else if constexpr (C == 1)
        {
            if constexpr (D == 0)
                return "десять";
            else if constexpr (D == 1)
                return "одиннадцать";
            else if constexpr (D == 2)
                return "двенадцать";
            else if constexpr (D == 3)
                return "тринадцать";
            else if constexpr (D == 4)
                return "четырнадцать";
            else if constexpr (D == 5)
                return "пятнадцать";
            else if constexpr (D == 6)
                return "шестнадцать";
            else if constexpr (D == 7)
                return "семнадцать";
            else if constexpr (D == 8)
                return "восемнадцать";
            else if constexpr (D == 9)
                return "девятнадцать";
        }
        else if constexpr (C == 4)
            return concatenate_with_delimiter(" ", {"сорок", unit});
        else if constexpr (C == 9)
            return concatenate_with_delimiter(" ", {"девяносто", unit});
            
        return concatenate_with_delimiter(" ", {Units<C>::unit() + name(), unit});
    }
    static std::string apply()
    {
        return ten();
    }
};

template <int A, int B, int C, bool from_thousands = false>
struct Hundreds
{
    static std::string name()
    {
        if constexpr (A == 0)
        {
            return "";
        }
        else if constexpr (A==1)
        {
            return "сто";
        }
        else if constexpr (A==2)
        {
            return "двести";
        }
        else if constexpr (A==3 || A==4)
        {
            return "ста";
        }
        else
        {
            return "сот";
        }
    }
    static std::string apply()
    {
        return concatenate_with_delimiter(" ", {Units<A>::from_hundreds() + name(), Tens<B, C, from_thousands>::apply()});
    }
};
} // namespace impl
} // namespace num

#endif
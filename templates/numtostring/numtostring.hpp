#ifndef NUMTOSTRING_HPP
#define NUMTOSTRING_HPP

#include "num_impl.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace num
{
using namespace impl;
template <int N>
struct DigitTraits;

template <>
struct DigitTraits<6>
{
    static constexpr std::string_view name = "миллион";
};

template <>
struct DigitTraits<3>
{
    static constexpr std::string_view name = "тысяч";
};

template <int N1, int N2, int N3, int... D>
struct ImplNumToString
{
    static constexpr int Order = sizeof...(D);
    static_assert(Order % 3 == 0, "Order must be a multiple of 3.");
    static_assert(has_digit_name<DigitTraits<Order>>::value, "Not implemented order");
    static std::string apply()
    {
        if (N1 == 0 && N2 == 0 && N3 == 0)
        {
            return ImplNumToString<D...>::apply();
        }
        std::string name(DigitTraits<Order>::name);
        std::string cur_h;
        if constexpr (Order != 3)
        {
            cur_h = Hundreds<N1, N2, N3>::apply();
            name += suffix<N2, N3>();
        }
        else
        {
            cur_h = Hundreds<N1, N2, N3, true>::apply();
            name += suffix<N2, N3, true>();
        }
        return concatenate_with_delimiter(" ", {cur_h, name, ImplNumToString<D...>::apply()});
    }
};

template <int N1, int N2, int N3>
struct ImplNumToString<N1, N2, N3>
{
    static std::string apply()
    {
        return Hundreds<N1, N2, N3>::apply();
    }
};

template <int... D>
struct NumToString
{
    static_assert(is_in_range<D...>(), "N must be in range [0, 9].");
    static std::string apply()
    {
        constexpr int size = sizeof...(D);
        if constexpr (all_zero<D...>())
        {
            return "ноль";
        }
        else if constexpr (size % 3 == 1)
        {
            return ImplNumToString<0, 0, D...>::apply();
        }
        else if constexpr (size % 3 == 2)
        {
            return ImplNumToString<0, D...>::apply();
        }
        else
        {
            return ImplNumToString<D...>::apply();
        }
    }
};
} // namespace num
#endif
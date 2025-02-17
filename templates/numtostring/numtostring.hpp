#ifndef NUMTOSTRING_HPP
#define NUMTOSTRING_HPP

#include "hundreds.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

template <int... D>
constexpr bool all_zero()
{
    return (... && (D == 0));
}

template <int T, int U, bool is_thousands = false>
static std::string suffix()
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
template <int N>
struct DigitTraits;

template <>
struct DigitTraits<12>
{
    static constexpr std::string_view name = "дециллион";
};

template <>
struct DigitTraits<11>
{
    static constexpr std::string_view name = "нониллион";
};

template <>
struct DigitTraits<10>
{
    static constexpr std::string_view name = "октиллион";
};

template <>
struct DigitTraits<9>
{
    static constexpr std::string_view name = "септиллион";
};

template <>
struct DigitTraits<8>
{
    static constexpr std::string_view name = "секстиллион";
};

template <>
struct DigitTraits<7>
{
    static constexpr std::string_view name = "квинтиллион";
};

template <>
struct DigitTraits<6>
{
    static constexpr std::string_view name = "квадриллион";
};

template <>
struct DigitTraits<5>
{
    static constexpr std::string_view name = "триллион";
};

template <>
struct DigitTraits<4>
{
    static constexpr std::string_view name = "миллиард";
};

template <>
struct DigitTraits<3>
{
    static constexpr std::string_view name = "миллион";
};

template <>
struct DigitTraits<2>
{
    static constexpr std::string_view name = "тысяч";
};

template <int Order, int N1, int N2, int N3, int... D>
struct ImplNumToString
{
    static std::string apply()
    {
        std::string name = std::string(DigitTraits<Order>::name);
        std::string cur_h;
        if constexpr (Order != 2)
        {
            cur_h = Hundreds<N1, N2, N3>::apply();
            name += suffix<N2, N3>();
        }
        else
        {
            cur_h = Hundreds<N1, N2, N3, true>::apply();
            name += suffix<N2, N3, true>();
        }
        return concatenate_with_delimiter(" ", {cur_h, name, ImplNumToString<Order - 1, D...>::apply()});
    }
};

template <int Order, int N1, int N2, int N3>
struct ImplNumToString<Order, N1, N2, N3>
{
    static std::string apply()
    {
        return concatenate_with_delimiter(" ", {Hundreds<N1, N2, N3>::apply()});
    }
};

template <int... D>
struct NumToString
{
    static std::string apply()
    {
        constexpr int size = sizeof...(D);
        if constexpr (all_zero<D...>())
        {
            return "ноль";
        }
        else if constexpr (size % 3 == 1)
        {
            return ImplNumToString<int(size / 3) + 1, 0, 0, D...>::apply();
        }
        else if constexpr (size % 3 == 2)
        {
            return ImplNumToString<int(size / 3) + 1, 0, D...>::apply();
        }
        else
        {
            return ImplNumToString<int(size / 3), D...>::apply();
        }
    }
};

#endif
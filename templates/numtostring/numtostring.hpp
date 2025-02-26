#ifndef NUMTOSTRING_HPP
#define NUMTOSTRING_HPP

#include "numtostringimpl.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace num
{
template <int N>
struct DigitTraits
{
	static constexpr std::string_view name = "";
};

template <>
struct DigitTraits<3>
{
    static constexpr std::string_view name = "тысяч";
};

template <int... D>
struct NumToString
{
    static std::string apply()
    {
        constexpr int size = sizeof...(D);
        if constexpr (impl::all_zero<D...>())
        {
            return "ноль";
        }
        else if constexpr (size % 3 == 1)
        {
            return impl::ImplNumToString<0, 0, D...>::apply();
        }
        else if constexpr (size % 3 == 2)
        {
            return impl::ImplNumToString<0, D...>::apply();
        }
        else
        {
            return impl::ImplNumToString<D...>::apply();
        }
    }
};


template <int N>
struct Int
{
    static std::string str()
    {
        if constexpr (N < 0)
        {
            return "минус " + Int<-N>::str(); 
        }
        else
        {
            return impl::NumToStringCaller<N>::apply();
        }
    }
};
} // namespace num
#endif
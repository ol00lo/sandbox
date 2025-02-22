#ifndef NUMTOSTRINGIMPL_HPP
#define NUMTOSTRINGIMPL_HPP

#include "numimpl.hpp"

namespace num
{
namespace impl
{

template <int N1, int N2, int N3, int... D>
struct ImplNumToString
{
    static constexpr int Order = sizeof...(D);
    static_assert(Order % 3 == 0, "Order must be a multiple of 3.");
    static_assert(DigitTraits<Order>::name != "", "Not implemented order");

    static std::string apply()
    {
        if constexpr (N1 == 0 && N2 == 0 && N3 == 0)
        {
            return ImplNumToString<D...>::apply();
        }
        else
        {
            std::string name(DigitTraits<Order>::name);
            std::string cur_h;
            if constexpr (Order != 3)
            {
                cur_h = impl::Hundreds<N1, N2, N3>::apply();
                name += impl::suffix<N2, N3>();
            }
            else
            {
                cur_h = impl::Hundreds<N1, N2, N3, true>::apply();
                name += impl::suffix<N2, N3, true>();
            }
            return impl::concatenate_with_delimiter(" ", {cur_h, name, ImplNumToString<D...>::apply()});
        }
    }
};

template <int N1, int N2, int N3>
struct ImplNumToString<N1, N2, N3>
{
    static std::string apply()
    {
        return impl::Hundreds<N1, N2, N3>::apply();
    }
};

template <int I0, int... I1>
struct NumToStringCaller
{
    static std::string apply()
    {
        if constexpr (I0 < 10)
        {
            return NumToString<I0, I1...>::apply();
        }
        else
        {
            return NumToStringCaller<I0 / 10, I0 % 10, I1...>::apply();
        }
    }
};
} // namespace impl
} // namespace num

#endif
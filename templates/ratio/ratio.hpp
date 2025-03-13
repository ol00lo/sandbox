#ifndef RATIO_HPP
#define RATIO_HPP
#include <iostream>
#include <string>

template <int A, int B>
constexpr int GCD()
{
    if constexpr (B == 0)
    {
        return A;
    }
    else
    {
        return GCD<B, A % B>();
    }
}

template <int Num, int Denum>
struct Ratio
{
    static_assert(Denum != 0, "Denominator cannot be zero");

    static constexpr int num = Denum < 0 ? -Num : Num;
    static constexpr int denum = Denum <0 ? -Denum : Denum;

    static std::string str()
    {
        return std::to_string(num) + "/" + std::to_string(denum);
    }
};

template <class R>
struct Simplify
{
    static constexpr int gcd = GCD<R::num, R::denum>();
    using type = Ratio<R::num / gcd, R::denum / gcd>;
};

template <typename R>
using ratio_simplify_t = typename Simplify<R>::type;

template <class R1, class R2>
struct Add
{
    using type = ratio_simplify_t<Ratio<R1::num * R2::denum + R2::num * R1::denum, R1::denum * R2::denum>>;
};

template <class R1, class R2>
struct Sub
{
    using type = ratio_simplify_t<Ratio<R1::num * R2::denum - R2::num * R1::denum, R1::denum * R2::denum>>;
};

template <class R1, class R2>
struct Mult
{
    using type = ratio_simplify_t<Ratio<R1::num * R2::num, R1::denum * R2::denum>>;
};

template <class R1, class R2>
struct Div
{
    static_assert(R2::num != 0, "Cannot divide by zero");
    using type = ratio_simplify_t<Ratio<R1::num * R2::denum, R1::denum * R2::num>>;
};

template <class R1, class R2>
struct Compare
{
    static constexpr int value = (R1::num * R2::denum < R2::num * R1::denum)   ? -1
                                 : (R1::num * R2::denum > R2::num * R1::denum) ? 1
                                                                               : 0;
};
template <typename R1, typename R2>
using ratio_add_t = typename Add<R1, R2>::type;

template <typename R1, typename R2>
using ratio_sub_t = typename Sub<R1, R2>::type;

template <typename R1, typename R2>
using ratio_mult_t = typename Mult<R1, R2>::type;

template <typename R1, typename R2>
using ratio_div_t = typename Div<R1, R2>::type;

template <typename R1, typename R2>
constexpr int ratio_compare_v = Compare<R1, R2>::value;

#endif
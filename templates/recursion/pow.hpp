#ifndef POW_HPP
#define POW_HPP
#include <iostream>

template <int N, typename Enable = void>
struct Pow
{
    static_assert(N >= -10 && N <= 10, "N should be in [-10, 10]");
    static constexpr double apply(double a)
    {
        return a * Pow<N - 1>::apply(a);
    }
};

template <int N>
struct Pow<N, typename std::enable_if<(N < 0)>::type>
{
    static constexpr double apply(double a)
    {
        double res = a * Pow<-N - 1>::apply(a);
        return 1 / res;
    }
};
template <>
struct Pow<0>
{
    static constexpr double apply(double a)
    {
        return 1;
    }
};

template <int I = 0>
double pow_apply(int N, double a)
{
    static_assert(I >= -10 && I <= 10, "I should be in [-10, 10]");

    if (N >= -10 && N <= 10)
    {
        if (I == N)
        {
            return Pow<I>::apply(a);
        }
        else if (I < N)
        {
            if constexpr (I < 10)
            {
                return pow_apply<I + 1>(N, a);
            }
            else
            {
                throw std::runtime_error("unreachable code");
            }
        }
        else
        {
            if constexpr (I > -10)
            {
                return pow_apply<I - 1>(N, a);
            }
            else
            {
                throw std::runtime_error("unreachable code");
            }
        }
    }
    else
    {
        throw std::runtime_error("N should be in [-10, 10]");
    }
}

double Pow_apply(int N, double a)
{
    switch (N)
    {
    case 1:
        return Pow<1>::apply(a);break;
    case 2:
        return Pow<2>::apply(a);break;
    case 3:
        return Pow<3>::apply(a);break;
    case 4:
        return Pow<4>::apply(a);break;
    case 5:
        return Pow<5>::apply(a);break;
    case 6:
        return Pow<6>::apply(a);break;
    case 7:
        return Pow<7>::apply(a);break;
    case 8:
        return Pow<8>::apply(a);break;
    case 9:
        return Pow<9>::apply(a);break;
    case 10:
        return Pow<10>::apply(a);break;
    case 0:
        return Pow<0>::apply(a);break;
    case -1:
        return Pow<-1>::apply(a);break;
    case -2:
        return Pow<-2>::apply(a);break;
    case -3:
        return Pow<-3>::apply(a);break;
    case -4:
        return Pow<-4>::apply(a);break;
    case -5:
        return Pow<-5>::apply(a);break;
    case -6:
        return Pow<-6>::apply(a);break;
    case -7:
        return Pow<-7>::apply(a);break;
    case -8:
        return Pow<-8>::apply(a);break;
    case -9:
        return Pow<-9>::apply(a);break;
    case -10:
        return Pow<-10>::apply(a);break;
    default:
        throw std::runtime_error("N should be in [-10, 10]");
    }
}

#endif
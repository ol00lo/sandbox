#ifndef MULTIPLY_HPP
#define MULTIPLY_HPP
#include <type_traits>
#include <iostream>

template <int A, int B, typename Enable = void>
struct Multiply
{
    static constexpr int value = A + Multiply<A, B - 1>::value;
};

template <int A, int B>
struct Multiply<A, B, typename std::enable_if<(B < 0)>::type>
{
    static constexpr int value = -(A + Multiply<A, -B - 1>::value);
};

template <int A>
struct Multiply<A, 1>
{
    static constexpr int value = A;
};

template <int A>
struct Multiply<A, 0>
{
    static constexpr int value = 0;
};


template <int A, int B, int C>
struct MultiplyThree
{
    static constexpr int value = Multiply<A, Multiply<B, C>::value>::value;
};


template <int A, int B>
struct Multiply_help1
{
    static_assert(A >= -10 && A <= 10, "A should be in [-10, 10]");
    static_assert(B >= -10 && B <= 10, "B should be in [-10, 10]");

    static constexpr int apply(int C)
    {
        switch (C)
        {
        case 10:
            return MultiplyThree<A, B, 10>::value;break;
        case 9:
            return MultiplyThree<A, B, 9>::value;break;
        case 8:
            return MultiplyThree<A, B, 8>::value;break;
        case 7:
            return MultiplyThree<A, B, 7>::value;break;
        case 6:
            return MultiplyThree<A, B, 6>::value;break;
        case 5:
            return MultiplyThree<A, B, 5>::value;break;
        case 4:
            return MultiplyThree<A, B, 4>::value;break;
        case 3:
            return MultiplyThree<A, B, 3>::value;break;
        case 2:
            return MultiplyThree<A, B, 2>::value;break;
        case 1:
            return MultiplyThree<A, B, 1>::value;break;
        case 0:
            return MultiplyThree<A, B, 0>::value;break;
        case -1:
            return MultiplyThree<A, B, -1>::value;break;
        case -2:
            return MultiplyThree<A, B, -2>::value;break;
        case -3:
            return MultiplyThree<A, B, -3>::value;break;
        case -4:
            return MultiplyThree<A, B, -4>::value;break;
        case -5:
            return MultiplyThree<A, B, -5>::value;break;
        case -6:
            return MultiplyThree<A, B, -6>::value;break;
        case -7:
            return MultiplyThree<A, B, -7>::value;break;
        case -8:
            return MultiplyThree<A, B, -8>::value;break;
        case -9:
            return MultiplyThree<A, B, -9>::value;break;
        case -10:
            return MultiplyThree<A, B, -10>::value;break;
        default:
            throw std::runtime_error("C should be in [-10, 10]");
        }
    }
};

template <int A>
struct Multiply_help2
{
    static_assert(A >= -10 && A <= 10, "A should be in [-10, 10]");
    static constexpr int apply(int B, int C)
    {
        switch (B)
        {
        case 10:
            return Multiply_help1<A, 10>::apply(C);break;
        case 9:
            return Multiply_help1<A, 9>::apply(C);break;
        case 8:
            return Multiply_help1<A, 8>::apply(C);break;
        case 7:
            return Multiply_help1<A, 7>::apply(C);break;
        case 6:
            return Multiply_help1<A, 6>::apply(C);break;
        case 5:
            return Multiply_help1<A, 5>::apply(C);break;
        case 4:
            return Multiply_help1<A, 4>::apply(C);break;
        case 3:
            return Multiply_help1<A, 3>::apply(C);break;
        case 2:
            return Multiply_help1<A, 2>::apply(C);break;
        case 1:
            return Multiply_help1<A, 1>::apply(C);break;
        case 0:
            return Multiply_help1<A, 0>::apply(C);break;
        case -1:
            return Multiply_help1<A, -1>::apply(C);break;
        case -2:
            return Multiply_help1<A, -2>::apply(C);break;
        case -3:
            return Multiply_help1<A, -3>::apply(C);break;
        case -4:
            return Multiply_help1<A, -4>::apply(C);break;
        case -5:
            return Multiply_help1<A, -5>::apply(C);break;
        case -6:
            return Multiply_help1<A, -6>::apply(C);break;
        case -7:
            return Multiply_help1<A, -7>::apply(C);break;
        case -8:
            return Multiply_help1<A, -8>::apply(C);break;
        case -9:
            return Multiply_help1<A, -9>::apply(C);break;
        case -10:
            return Multiply_help1<A, -10>::apply(C);break;
        default:
            throw std::runtime_error("B should be in [-10, 10]");
        }
    }
};

int Multiply_apply(int A, int B, int C = 1)
{
    switch (A)
    {
    case 10:
        return Multiply_help2<10>::apply(B, C);break;
    case 9:
        return Multiply_help2<9>::apply(B, C);break;
    case 8:
        return Multiply_help2<8>::apply(B, C);break;
    case 7:
        return Multiply_help2<7>::apply(B, C);break;
    case 6:
        return Multiply_help2<6>::apply(B, C);break;
    case 5:
        return Multiply_help2<5>::apply(B, C);break;
    case 4:
        return Multiply_help2<4>::apply(B, C);break;
    case 3:
        return Multiply_help2<3>::apply(B, C);break;
    case 2:
        return Multiply_help2<2>::apply(B, C);break;
    case 1:
        return Multiply_help2<1>::apply(B, C);break;
    case 0:
        return Multiply_help2<0>::apply(B, C);break;
    case -1:
        return Multiply_help2<-1>::apply(B, C);break;
    case -2:
        return Multiply_help2<-2>::apply(B, C);break;
    case -3:
        return Multiply_help2<-3>::apply(B, C);break;
    case -4:
        return Multiply_help2<-4>::apply(B, C);break;
    case -5:
        return Multiply_help2<-5>::apply(B, C);break;
    case -6:
        return Multiply_help2<-6>::apply(B, C);break;
    case -7:
        return Multiply_help2<-7>::apply(B, C);break;
    case -8:
        return Multiply_help2<-8>::apply(B, C);break;
    case -9:
        return Multiply_help2<-9>::apply(B, C);break;
    case -10:
        return Multiply_help2<-10>::apply(B, C);break;
    default:
        throw std::runtime_error("A should be in [-10, 10]");
    }
}

#endif
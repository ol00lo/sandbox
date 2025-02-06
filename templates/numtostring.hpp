#include <iostream>
#include <stdexcept>
#include <string>

template <int D>
struct Units
{
    static std::string unit()
    {
        if constexpr (D == 0)
            return " ";
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

    static std::string from_thousands()
    {
        if constexpr (D == 1)
            return "одна";
        else
            return from_hundreds();
    }
    static std::string from_hundreds()
    {
        if constexpr (D == 1 || D == 0)
            return "";
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

template <int C, int D>
struct Tens
{
    static constexpr std::string name()
    {
        if constexpr (C == 2 || C == 3)
        {
            return "дцать ";
        }
        else
        {
            return "десят ";
        }
    }

    static std::string ten()
    {
        if constexpr (C == 0)
            return Units<D>::unit();
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
            return "сорок " + Units<D>::unit();
        else if constexpr (C == 9)
            return "девяносто " + Units<D>::unit();

        return Units<C>::unit() + name() + Units<D>::unit();
    }

    static std::string apply()
    {
        return ten();
    }
};

template <int B>
struct Hundreds
{
    static std::string name()
    {
        if constexpr (B == 0)
        {
            return "";
        }
        else if constexpr (B==1)
        {
            return "сто ";
        }
        else if constexpr (B==2)
        {
            return "сти ";
        }
        else if constexpr (B==3 || B==4)
        {
            return "ста ";
        }
        else
        {
            return "сот ";
        }
    }
   
    static std::string hundred()
    {
        return Units<B>::from_hundreds() + name();
    }

    static std::string apply()
    {
        return hundred();
    }
};

template <int A>
struct Thousands
{
    static constexpr std::string name()
    {
        if constexpr (A == 0)
        {
            return "";
        }
        else if constexpr (A == 1)
        {
            return " тысяча ";
        }
        else if constexpr (A == 2 || A == 3 || A == 4)
        {
            return " тысячи ";
        }
        else
        {
            return " тысяч ";
        }
    }
    
    static std::string thousand()
    {
        return Units<A>::from_thousands() + name();
    }
    static std::string apply()
    {
        return thousand();
    }
};

template <int A = -1, int B = -1, int C = -1, int D = -1>
struct NumToString
{
    static_assert(A >= 0 && A < 10, "Invalid A");
    static_assert(B >= 0 && B < 10, "Invalid B");
    static_assert(C >= 0 && C < 10, "Invalid C");
    static_assert(D >= 0 && D < 10, "Invalid D");
    static std::string apply()
    {
        return Thousands<A>::apply() + Hundreds<B>::apply() + Tens<C, D>::apply();
    }
};

template <int A, int B, int C>
struct NumToString<A, B, C, -1> : public NumToString<0, A, B, C>{};
template <int A, int B>
struct NumToString<A, B, -1, -1> : public NumToString<0, 0, A, B>{};
template <int A>
struct NumToString<A, -1, -1, -1> : public NumToString<0, 0, 0, A>{};

template <>
struct NumToString<0, 0, 0, 0>
{
    static std::string apply()
    {
        return "ноль";
    }
};
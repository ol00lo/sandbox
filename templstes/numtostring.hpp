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
            return "����";
        else if constexpr (D == 2)
            return "���";
        else if constexpr (D == 3)
            return "���";
        else if constexpr (D == 4)
            return "������";
        else if constexpr (D == 5)
            return "����";
        else if constexpr (D == 6)
            return "�����";
        else if constexpr (D == 7)
            return "����";
        else if constexpr (D == 8)
            return "������";
        else if constexpr (D == 9)
            return "������";
    }

    static std::string from_thousands()
    {
        if constexpr (D == 1)
            return "����";
        else
            return from_hundreds();
    }
    static std::string from_hundreds()
    {
        if constexpr (D == 1 || D == 0)
            return "";
        else if constexpr (D == 2)
            return "���";
        else
            return unit();
    }
};

template <int C, int D>
struct Tens
{
    template <int I, class Enable = void>
    struct Name
    {
        static std::string apply()
        {
            return "����� ";
        }
    };
    template <int I>
    struct Name<I, std::enable_if_t<I == 2 || I == 3, void>>
    {
        static std::string apply()
        {
            return "����� ";
        }
    };
    static std::string ten()
    {
        if constexpr (C == 0)
            return Units<D>::unit();
        else if constexpr (C == 1)
        {
            if constexpr (D == 0)
                return "������";
            else if constexpr (D == 1)
                return "�����������";
            else if constexpr (D == 2)
                return "����������";
            else if constexpr (D == 3)
                return "����������";
            else if constexpr (D == 4)
                return "������������";
            else if constexpr (D == 5)
                return "����������";
            else if constexpr (D == 6)
                return "�����������";
            else if constexpr (D == 7)
                return "����������";
            else if constexpr (D == 8)
                return "������������";
            else if constexpr (D == 9)
                return "������������";
        }
        else if constexpr (C == 4)
            return "����� " + Units<D>::unit();
        else if constexpr (C == 9)
            return "��������� " + Units<D>::unit();

        return Units<C>::unit() + Name<C>::apply() + Units<D>::unit();
    }
};

template <int B, int C, int D>
struct Hundreds
{
    template <int I, class Enable = void>
    struct Name
    {
        static std::string apply()
        {
            return "��� ";
        }
    };
    template <>
    struct Name<1, void>
    {
        static std::string apply()
        {
            return "��� ";
        }
    };
    template <>
    struct Name<0, void>
    {
        static std::string apply()
        {
            return "";
        }
    };
    template <>
    struct Name<2, void>
    {
        static std::string apply()
        {
            return "��� ";
        }
    };
    template <int I>
    struct Name<I, std::enable_if_t<I == 3 || I == 4, void>>
    {
        static std::string apply()
        {
            return "��� ";
        }
    };
    static std::string hundred()
    {
        return Units<B>::from_hundreds() + Name<B>::apply() + Tens<C, D>::ten();
    }
};

template <int A, int B, int C, int D>
struct Thousands
{
    template <int I, class Enable = void>
    struct Name
    {
        static std::string apply()
        {
            return " ����� ";
        }
    };
    template <>
    struct Name<0, void>
    {
        static std::string apply()
        {
            return "";
        }
    };
    template <>
    struct Name<1, void>
    {
        static std::string apply()
        {
            return " ������ ";
        }
    };
    template <int I>
    struct Name<I, std::enable_if_t<I == 2 || I == 3 || I == 4, void>>
    {
        static std::string apply()
        {
            return " ������ ";
        }
    };

    static std::string thousand()
    {
        return Units<A>::from_thousands() + Name<A>::apply() + Hundreds<B, C, D>::hundred();
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
        return Thousands<A, B, C, D>::thousand();
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
        return "����";
    }
};
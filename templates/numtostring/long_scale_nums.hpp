#ifndef LONG_SCALE_NUMS
#define LONG_SCALE_NUMS
#include "numtostring.hpp"

using namespace num;

template <>
struct DigitTraits<33>
{
    static constexpr std::string_view name = "квинтиллиард";
};

template <>
struct DigitTraits<30>
{
    static constexpr std::string_view name = "квинтиллион";
};

template <>
struct DigitTraits<27>
{
    static constexpr std::string_view name = "квадриллиард";
};

template <>
struct DigitTraits<24>
{
    static constexpr std::string_view name = "квадриллион";
};

template <>
struct DigitTraits<21>
{
    static constexpr std::string_view name = "триллиард";
};

template <>
struct DigitTraits<18>
{
    static constexpr std::string_view name = "триллион";
};

template <>
struct DigitTraits<15>
{
    static constexpr std::string_view name = "биллиард";
};

template <>
struct DigitTraits<12>
{
    static constexpr std::string_view name = "биллион";
};

template <>
struct DigitTraits<9>
{
    static constexpr std::string_view name = "миллиард";
};

#endif 
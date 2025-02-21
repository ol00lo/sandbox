#ifndef SHORT_SCALE_NUMS
#define SHORT_SCALE_NUMS
#include "numtostring.hpp"

using namespace num;

template <>
struct DigitTraits<33>
{
    static constexpr std::string_view name = "дециллион";
};

template <>
struct DigitTraits<30>
{
    static constexpr std::string_view name = "нониллион";
};

template <>
struct DigitTraits<27>
{
    static constexpr std::string_view name = "октиллион";
};

template <>
struct DigitTraits<24>
{
    static constexpr std::string_view name = "септиллион";
};

template <>
struct DigitTraits<21>
{
    static constexpr std::string_view name = "секстиллион";
};

template <>
struct DigitTraits<18>
{
    static constexpr std::string_view name = "квинтиллион";
};

template <>
struct DigitTraits<15>
{
    static constexpr std::string_view name = "квадриллион";
};

template <>
struct DigitTraits<12>
{
    static constexpr std::string_view name = "триллион";
};

template <>
struct DigitTraits<9>
{
    static constexpr std::string_view name = "биллион";
};


#endif
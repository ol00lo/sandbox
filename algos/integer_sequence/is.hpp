#ifndef IS_HPP
#define IS_HPP

#include <array>
#include <iostream>
#include <utility>

namespace impl
{
template <int Start, int End, int Step>
using is_valid = typename std::enable_if<(Step > 0 ? Start < End : End < Start)>::type;

template <int Start, int End, int Step>
using is_end = typename std::enable_if<(Step > 0 ? Start >= End : End >= Start)>::type;

template <int Step>
using is_step = typename std::enable_if<Step == 0>::type;

template <typename Enable = void, int Start = 0, int End = 0, int Step = 0, int... Indices>
struct make_sequence_impl;

template <int Start, int End, int Step, int... Indices>
struct make_sequence_impl<is_valid<Start, End, Step>, Start, End, Step, Indices...>
    : make_sequence_impl<void, Start + Step, End, Step, Indices..., Start> {};

template <int Start, int End, int Step, int... Indices>
struct make_sequence_impl<is_end<Start, End, Step>, Start, End, Step, Indices...>
{
    using type = std::integer_sequence<int, Indices...>;
};

template <int Start, int End>
struct make_sequence_impl<void, Start, End, 0>
{
    using type = std::false_type;
};
} // namespace impl


// ========= operator<<
template <typename T, T... I>
std::ostream& operator<<(std::ostream& os, const std::integer_sequence<T, I...>&)
{
    os << "< ";
    ((os << I << " "), ...);
    os << ">";
    return os;
}

// ========= make sequence

template <int Start, int End, int Step = 1>
using make_sequence = typename impl::make_sequence_impl<void, Start, End, Step>::type;

// ========= sequence entry
template <int I, int... S>
constexpr int sequence_entry(std::integer_sequence<int, S...>)
{
    static_assert(I >= 0 && I < sizeof...(S), "Index out of bounds");
    auto seq = std::array<int, sizeof...(S)>{S...};
    return std::get<I>(seq);
}

template <int... S>
constexpr int sequence_entry(std::integer_sequence<int, S...>, int index)
{
    if (index < 0 || index >= sizeof...(S))
    {
        throw std::runtime_error("Index out of bounds");
    }
    auto seq = std::array<int, sizeof...(S)>{S...};
    return seq[index];
}

#endif
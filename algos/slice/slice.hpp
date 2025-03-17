#ifndef SLICE_HPP
#define SLICE_HPP

#include <iostream>
#include <string_view>
#include <tuple>
#include <utility>

template <class T>
constexpr std::string_view type_name()
{
    using namespace std;
#ifdef __clang__
    string_view p = __PRETTY_FUNCTION__;
    return string_view(p.data() + 34, p.size() - 34 - 1);
#elif defined(__GNUC__)
    string_view p = __PRETTY_FUNCTION__;
    return string_view(p.data() + 49, p.find(';', 49) - 49);
#elif defined(_MSC_VER)
    string_view p = __FUNCSIG__;
    return string_view(p.data() + 84, p.size() - 84 - 7);
#endif
}

namespace impl
{
// ------- make sequence -------

template <int Start, int End, int Step>
using is_valid = typename std::enable_if<(Step > 0 ? Start < End : End < Start)>::type;

template <int Start, int End, int Step>
using is_end = typename std::enable_if<(Step > 0 ? Start >= End : End >= Start)>::type;

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

template <int Start, int End, int Step = 1>
using make_sequence = typename make_sequence_impl<void, Start, End, Step>::type;

// ------- tuple slice -------

template <class Tuple, size_t... I>
struct TupleSlice
{
    using type_t = std::tuple<std::tuple_element_t<I, Tuple>...>;

    static type_t apply(Tuple t)
    {
        return type_t{std::get<I>(t)...};
    }
};

// ------- tuple slice range -------

template <class Tuple, int... I>
TupleSlice<Tuple, I...> build_tuple_slice(std::integer_sequence<int, I...>);

template <class Tuple, class Seq>
using TupleSliceRange = decltype(build_tuple_slice<Tuple>(Seq{}));

} // namespace impl


template <int... I, class Tuple>
auto make_tuple_slice(Tuple t)
{
    return impl::TupleSlice<Tuple, I...>::apply(t);
}

template <int I1, int I2, class Tuple>
auto make_tuple_slice_range(const Tuple& t)
{
    return impl::TupleSliceRange<Tuple, impl::make_sequence<I1, I2>>::apply(t);
}

#endif // SLICE_HPP
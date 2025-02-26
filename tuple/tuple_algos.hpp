#ifndef TUPLE_ALGOS_H
#define TUPLE_ALGOS_H
#include <iostream>
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

// ======== for each
template <int I = 0, class Tuple, class Operator>
void for_each(Tuple& t, Operator oper)
{
    constexpr size_t size = std::tuple_size_v<Tuple>;
    if constexpr (I < size)
    {
        oper(std::get<I>(t));
        for_each<I + 1>(t, oper);
    }
}

template <class Tuple, class Operator, int... I>
void for_each_impl(Tuple& t, Operator oper, std::integer_sequence<int, I...>)
{
    (oper(std::get<I>(t)), ...);
}
template <class Tuple, class Operator>
void for_each_fe(Tuple& t, Operator oper)
{
    using seq_t = std::make_integer_sequence<int, std::tuple_size_v<Tuple>>;
    for_each_impl(t, oper, seq_t());
}

// ======== any of
template <std::size_t Index = 0, class Tuple, class Operator>
bool any_of(const Tuple& t, Operator oper)
{
    if constexpr (Index < std::tuple_size_v<Tuple>)
    {
        return (oper(std::get<Index>(t))) ? true : any_of<Index + 1>(t, oper);
    }
    return false;
}

template <class Tuple, class Operator, std::size_t... Is>
bool any_of_impl(const Tuple& t, Operator oper, std::index_sequence<Is...>)
{
    return (oper(std::get<Is>(t)) || ...);
}

template <class Tuple, class Operator>
bool any_of_fe(const Tuple& t, Operator oper)
{
    using seq_t = std::make_index_sequence<std::tuple_size_v<Tuple>>;
    return any_of_impl(t, oper, seq_t());
}

// ======== transform
template <class Oper, class... T>
struct TransformT
{
    using type_t = std::tuple<std::invoke_result_t<Oper, T>...>;
};

template <int N, typename newTuple, typename Tuple, typename Oper>
void fill_tuple(newTuple& ret, const Tuple& t, Oper oper)
{
    std::get<N>(ret) = oper(std::get<N>(t));
    if constexpr (N > 0)
    {
        fill_tuple<N - 1>(ret, t, oper);
    }
}

template <typename T, typename Oper, int... I>
auto transform_impl(const T& t, Oper&& oper, std::integer_sequence<int, I...> seq)
{
    return std::make_tuple(oper(std::get<I>(t))...);
}

template <typename Oper, class... T>
typename TransformT<Oper, T...>::type_t transform(const std::tuple<T...>& t, Oper&& oper)
{
    using in_t = std::tuple<T...>;
    using ret_t = typename TransformT<Oper, T...>::type_t;
    constexpr size_t size = std::tuple_size_v<in_t>;
    ret_t ret;
    fill_tuple<size - 1>(ret, t, oper);

    return ret;
}

template <typename Oper, class... T>
typename TransformT<Oper, T...>::type_t transform_fe(const std::tuple<T...>& t, Oper&& oper)
{
    using in_t = std::tuple<T...>;
    using ret_t = typename TransformT<Oper, T...>::type_t;
    constexpr size_t size = std::tuple_size_v<in_t>;
    auto seq = std::make_integer_sequence<int, size>();
    ret_t ret = transform_impl(t, std::forward<Oper>(oper), seq);
    return ret;
}

#endif
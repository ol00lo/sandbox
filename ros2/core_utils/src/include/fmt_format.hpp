#ifndef FMT_FORMAT_HPP
#define FMT_FORMAT_HPP

#include <fmt/core.h>
#include <fmt/format.h>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>

namespace detail
{
template <typename T>
struct is_fmt_view : std::is_base_of<fmt::detail::view, std::remove_cv_t<std::remove_reference_t<T>>> {};

template <typename T, typename std::enable_if<!is_fmt_view<T>::value, int>::type = 0>
std::string arg_to_string(const T& arg)
{
    std::ostringstream oss;
    oss << arg;
    return oss.str();
}

template <typename Char, typename T, typename std::enable_if<!is_fmt_view<T>::value, int>::type = 0>
std::string arg_to_string(const fmt::detail::named_arg<Char, T>& arg)
{
    std::ostringstream oss;
    oss << "{\"" << arg.name << "\" => " << arg.value << "}";
    return oss.str();
}

template <typename T, typename std::enable_if<is_fmt_view<T>::value, int>::type = 0>
std::string arg_to_string(T arg)
{
    return fmt::format("{}", std::move(arg));
}

template <typename... Args>
std::string make_error_message(std::string_view format_str, Args&&... args)
{
    std::ostringstream oss;
    oss << "Failed to perform format substitution using the format string \"" << format_str
        << "\" and the placeholders: ";

    size_t n = 0;
    ((oss << (n++ > 0 ? ", " : "") << arg_to_string(args)), ...);

    return oss.str();
}
}

template <typename... Args>
std::string fmt_format(std::string_view format_str, Args&&... args)
{
    try
    {
        return fmt::format(format_str, std::forward<Args>(args)...);
    }
    catch (const fmt::format_error& e)
    {
        auto error_msg = detail::make_error_message(format_str, std::forward<Args>(args)...);
        throw std::runtime_error(error_msg);
    }
}

#endif
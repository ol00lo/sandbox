#ifndef CORE_UTILS_HPP
#define CORE_UTILS_HPP

#include <fmt/core.h>
#include <fmt/format.h>
#include <sstream>
#include <string>
#include <string_view>


template <typename T>
std::string arg_to_string(const T& arg)
{
    std::ostringstream oss;
    oss << arg;
    return oss.str();
}

template <typename Char, typename T>
std::string arg_to_string(const fmt::detail::named_arg<Char, T>& arg)
{
    std::ostringstream oss;
    oss << "{\"" << arg.name << "\" => " << arg.value << "}";
    return oss.str();
}

template <typename... Args>
std::string make_error_message(std::string_view format_str, Args&&... args)
{
    std::ostringstream oss;
    oss << "Failed to perform format substitution using the format string \"" << format_str
        << "\" and the placeholders: ";

    std::size_t n = 0;
    ((oss << (n++ > 0 ? ", " : "") << arg_to_string(args)), ...);

    return oss.str();
}

template <typename... Args>
std::string fmt_format(std::string_view format_str, Args&&... args)
{
    try
    {
        return fmt::vformat(format_str, fmt::make_format_args(args ...));
    } 
    catch (const fmt::format_error& e)
    {
        auto error_msg = make_error_message(format_str, std::forward<Args>(args)...);
        throw std::runtime_error(error_msg);
    }
}

#endif
#include "core_utils/fmt_format.hpp"
#include <fmt/format.h>
#include <gtest/gtest.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
std::string get_error_string(std::string s1, std::string s2)
{
    return "Failed to perform format substitution using the format string " + s1 + " and the placeholders: " + s2;
}
}

TEST(CoreUtils, ArgsToStringTest)
{
    EXPECT_EQ(detail::arg_to_string(42), std::string("42"));
    EXPECT_EQ(detail::arg_to_string(12.2), std::string("12.2"));
    EXPECT_EQ(detail::arg_to_string("a"), "a");
    auto a = fmt::arg("x", 12);
    EXPECT_EQ(detail::arg_to_string(a), std::string("{\"x\" => 12}"));
    auto a1 = fmt::arg("x", 12.2);
    EXPECT_EQ(detail::arg_to_string(a1), std::string("{\"x\" => 12.2}"));
}

TEST(CoreUtils, MakeErrorTest)
{
    std::string msg = detail::make_error_message("{} {}", 1, 2);
    EXPECT_NE(msg.find("Failed to perform format substitution"), std::string::npos);
    EXPECT_NE(msg.find("\"{} {}\""), std::string::npos);
    EXPECT_NE(msg.find("1, 2"), std::string::npos);
}

TEST(FmtFormat, FormatsOnSuccess)
{
    EXPECT_EQ(fmt_format("Hello {} {}", "World", 123), "Hello World 123");
    EXPECT_EQ(fmt_format("Number: {}", 42), "Number: 42");
    EXPECT_EQ(fmt_format("Pi: {:.2f}", 3.14159), "Pi: 3.14");
    auto result = fmt_format("Hello {name} {value} {name}", fmt::arg("name", "Test"), fmt::arg("value", 456));
    EXPECT_EQ(result, "Hello Test 456 Test");
    std::vector<int> v = {10, 15, 5, 10, 5, 4, 5, 10, 5};
    EXPECT_EQ(fmt_format("minutes {} am", fmt::join(v, " - ")), "minutes 10 - 15 - 5 - 10 - 5 - 4 - 5 - 10 - 5 am");
}

TEST(FmtFormat, FailTests)
{
    try
    {
        fmt_format("{} {} {}", 1, 2);
        FAIL() << "Expected std::runtime_error";
    }
    catch (const std::runtime_error& e)
    {
        EXPECT_EQ(e.what(), get_error_string("\"{} {} {}\"", "1, 2"));
    }

    try
    {
        fmt_format("Number: {:.2q}", 42);
        FAIL() << "Expected std::runtime_error";
    }
    catch (const std::runtime_error& e)
    {
        EXPECT_EQ(e.what(), get_error_string("\"Number: {:.2q}\"", "42"));
    }

    try
    {
        fmt_format("Number: {a}, {b}, {c}", fmt::arg("a", 1), fmt::arg("b", 2), fmt::arg("d", 3));
        FAIL() << "Expected std::runtime_error";
    }
    catch (const std::runtime_error& e)
    {
        EXPECT_EQ(e.what(), get_error_string("\"Number: {a}, {b}, {c}\"", "{\"a\" => 1}, {\"b\" => 2}, {\"d\" => 3}"));
    }

    try
    {
        fmt_format("{} {} {b}", 1, 2, fmt::arg("a", 12));
        FAIL() << "Expected std::runtime_error";
    }
    catch (const std::runtime_error& e)
    {
        EXPECT_EQ(e.what(), get_error_string("\"{} {} {b}\"", "1, 2, {\"a\" => 12}"));
    }

    try
    {
        fmt_format("{} - empty");
        FAIL() << "Expected std::runtime_error";
    }
    catch (const std::runtime_error& e)
    {
        EXPECT_EQ(e.what(), get_error_string("\"{} - empty\"", ""));
    }
}
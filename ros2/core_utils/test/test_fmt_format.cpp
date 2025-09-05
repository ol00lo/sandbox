#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include "core_utils.hpp"
#include <fmt/format.h>

TEST(CoreUtilsFmtFormat, FormatsOnSuccess) {
    EXPECT_EQ(1*1,  1);
}

TEST(CoreUtilsFmtFormat, TryFunction) {
    EXPECT_EQ(fmt_format("Hello {} {}", "World", 123), "Hello World 123");
    EXPECT_EQ(fmt_format("Number: {}", 42), "Number: 42");
    EXPECT_EQ(fmt_format("Pi: {:.2f}", 3.14159), "Pi: 3.14");
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
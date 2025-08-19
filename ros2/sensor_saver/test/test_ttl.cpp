#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <pqxx/pqxx>
#include "sensor_saver_impl.hpp"

TEST(SimpleTest, BasicAssertion) {
  EXPECT_EQ(1, 1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
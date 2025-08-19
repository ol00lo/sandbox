#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <pqxx/pqxx>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include "sensor_saver_impl.hpp"

TEST(SimpleTest, BasicAssertion) {
  EXPECT_EQ(1, 1);
}

TEST(TTLtest, UpdatesTTL) {
  std::string conn_str = "host=db dbname=mydb user=user password=password port=5432";
  int ttl_days = 1;
  std::string table_name = "mouse_movements_ttl_test";

  SensorSaverImpl impl(conn_str, ttl_days, rclcpp::get_logger("ttl_test"));
  impl.init_db(table_name);
  impl.clear_table();

  const auto now_tp = std::chrono::system_clock::now();
  const long now_seconds = std::chrono::system_clock::to_time_t(now_tp);
  const double old_ts = now_seconds - 2 * 24 * 60 * 60;
  const double recent_ts = now_seconds - 1 * 60 * 60;
  impl.save_to_db(0.0, 0.0, old_ts);
  impl.save_to_db(1.0, 1.0, recent_ts);

  EXPECT_EQ(impl.count_rows(), (size_t)(2));

  impl.cleanup_old_data();
  EXPECT_EQ(impl.count_rows(), (size_t)(1));

  impl.update_ttl(0);
  impl.cleanup_old_data();
  EXPECT_EQ(impl.count_rows(), (size_t)(0));

  impl.clear_table();
  rclcpp::shutdown();
}

TEST(TTLwithTimer, UpdatesTTL) {
  std::string conn_str = "host=db dbname=mydb user=user password=password port=5432";
  int ttl_days = 0;
  std::string table_name = "mouse_movements_ttl_test";

  SensorSaverImpl impl(conn_str, ttl_days, rclcpp::get_logger("ttl_test"));
  impl.init_db(table_name);
  impl.clear_table();

  const auto now_tp = std::chrono::system_clock::now();
  const auto now_seconds = static_cast<long>(std::chrono::system_clock::to_time_t(now_tp));
  const double old_ts = static_cast<double>(now_seconds - 2 * 24 * 60 * 60);
  impl.save_to_db(0.0, 0.0, old_ts);
  EXPECT_EQ(impl.count_rows(), static_cast<std::size_t>(1));

  impl.start_cleanup_timer(std::chrono::hours(0));
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(impl.count_rows(), static_cast<std::size_t>(0));
  impl.stop_cleanup_timer();

  impl.clear_table();
  rclcpp::shutdown();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
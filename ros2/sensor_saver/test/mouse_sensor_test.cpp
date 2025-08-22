#include "sensor_saver_impl.hpp"
#include <chrono>
#include <cstdlib>
#include <gtest/gtest.h>
#include <pqxx/pqxx>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace {
class MouseSensor : public ::testing::Test {
public:
    std::unique_ptr<SensorSaverImpl> impl;
    long now_seconds;

protected:
    void SetUp() override {
        const std::string conn_str = "host=localhost user=test_user password=test_pass dbname=test_db";
        std::string table_name = "mouse_movements_ttl_test";
        impl = std::make_unique<SensorSaverImpl>(conn_str, 1, rclcpp::get_logger("test_logger"), table_name);
        impl->clear_table();

        now_seconds = static_cast<long>(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    }
    void TearDown() override {
        impl->clear_table();
    }
};
}

TEST_F(MouseSensor, TtlTest) {
    impl->update_ttl(1);

    const auto old_tp = std::chrono::system_clock::time_point{std::chrono::seconds{now_seconds - 2 * 24 * 60 * 60}};
    const auto recent_tp = std::chrono::system_clock::time_point{std::chrono::seconds{now_seconds - 1 * 60 * 60}};
    impl->save_to_db(0.0, 0.0, old_tp);
    impl->save_to_db(1.0, 1.0, recent_tp);

    EXPECT_EQ(impl->rows_count(), 2ul);

    impl->cleanup_old_data();
    EXPECT_EQ(impl->rows_count(), 1ul);

    impl->update_ttl(0);
    impl->cleanup_old_data();
    EXPECT_EQ(impl->rows_count(), 0ul);
}

TEST_F(MouseSensor, TtlWithTimerTest) {
    impl->update_ttl(0);

    const auto old_tp = std::chrono::system_clock::time_point{std::chrono::seconds{now_seconds - 2 * 24 * 60 * 60}};
    impl->save_to_db(0.0, 0.0, old_tp);
    EXPECT_EQ(impl->rows_count(), 1ul);

    impl->start_cleanup_timer(std::chrono::hours(0));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_EQ(impl->rows_count(), 0ul);
    impl->stop_cleanup_timer();
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
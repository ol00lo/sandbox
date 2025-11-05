#include "sensor_saver_impl/sensor_saver_impl.hpp"
#include <chrono>
#include <cstdlib>
#include <gtest/gtest.h>
#include <pqxx/pqxx>
#include <rclcpp/rclcpp.hpp>
#include <thread>

class SensorSaverImpl_Tester{
public:
    static size_t rows_count(const std::unique_ptr<SensorSaverImpl>& pimpl){
        pqxx::work txn(*pimpl->db_conn_);
        std::string sql = "SELECT COUNT(*) FROM " + txn.quote_name(pimpl->table_name_);
        auto res = txn.exec(sql);
        txn.commit();
        return res[0][0].as<size_t>(0);
    }

    static void clear_table(const std::unique_ptr<SensorSaverImpl>& pimpl){
        pqxx::work txn(*pimpl->db_conn_);
        txn.exec("TRUNCATE TABLE " + txn.quote_name(pimpl->table_name_) + " RESTART IDENTITY;");
        txn.commit();
    }

    static void cleanup_old_data(const std::unique_ptr<SensorSaverImpl>& pimpl){
        pimpl->cleanup_old_data();
    }
};

namespace {
class MouseSensor : public ::testing::Test {
public:
    std::unique_ptr<SensorSaverImpl> impl;
    int64_t now_seconds;

protected:
    void SetUp() override {
        std::string table_name = "mouse_movements_ttl_test";
        impl = std::make_unique<SensorSaverImpl>(DbConnectionSettings{"db", "mydb", "user", "password", 5432}, 1, rclcpp::get_logger("test_logger"), table_name);
        SensorSaverImpl_Tester::clear_table(impl);

        now_seconds = static_cast<int64_t>(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    }
    void TearDown() override {
        SensorSaverImpl_Tester::clear_table(impl);
    }
};
}

TEST_F(MouseSensor, TtlTest) {
    impl->update_ttl(1);

    const auto old_tp = std::chrono::system_clock::time_point{std::chrono::seconds{now_seconds - 2 * 24 * 60 * 60}};
    const auto recent_tp = std::chrono::system_clock::time_point{std::chrono::seconds{now_seconds - 1 * 60 * 60}};
    impl->save_to_db(0.0, 0.0, old_tp);
    impl->save_to_db(1.0, 1.0, recent_tp);

    EXPECT_EQ(SensorSaverImpl_Tester::rows_count(impl), (size_t)2);

    SensorSaverImpl_Tester::cleanup_old_data(impl);
    EXPECT_EQ(SensorSaverImpl_Tester::rows_count(impl), (size_t)1);

    impl->update_ttl(0);
    SensorSaverImpl_Tester::cleanup_old_data(impl);
    EXPECT_EQ(SensorSaverImpl_Tester::rows_count(impl), (size_t)0);
}

TEST_F(MouseSensor, TtlWithTimerTest) {
    impl->update_ttl(0);

    const auto old_tp = std::chrono::system_clock::time_point{std::chrono::seconds{now_seconds - 2 * 24 * 60 * 60}};
    impl->save_to_db(0.0, 0.0, old_tp);
    EXPECT_EQ(SensorSaverImpl_Tester::rows_count(impl), (size_t)1);

    impl->start_cleanup_timer(std::chrono::hours(0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(SensorSaverImpl_Tester::rows_count(impl), (size_t)0);
    impl->stop_cleanup_timer();
}
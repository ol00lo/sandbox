#ifndef SENSOR_SAVER_IMPL_HPP
#define SENSOR_SAVER_IMPL_HPP

#include <pqxx/pqxx>
#include <string>
#include <cstddef>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <rclcpp/rclcpp.hpp>

class SensorSaverImpl {
public:
    SensorSaverImpl(const std::string& conn_str, int64_t ttl_days, rclcpp::Logger logger,
                    const std::string& table_name = "mouse_movements");

    void cleanup_old_data();
    void update_ttl(int64_t ttl_days_new);
    void save_to_db(double x, double y, std::chrono::system_clock::time_point timestamp);
    size_t rows_count() const;

    void start_cleanup_timer(std::chrono::hours time);
    void change_cleanup_interval(std::chrono::hours new_interval);
    void stop_cleanup_timer();
    void clear_table();

private:
    std::string conn_str_;
    int64_t ttl_days_;
    rclcpp::Logger logger_;
    const std::string table_name_;
    std::unique_ptr<pqxx::connection> db_conn_;

    std::atomic<bool> cleanup_active_ = false;
    std::thread cleanup_thread_;
};


#endif
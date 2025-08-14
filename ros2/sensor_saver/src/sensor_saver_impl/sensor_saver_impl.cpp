#include "sensor_saver_impl.hpp"

SensorSaverImpl::SensorSaverImpl(const std::string& conn_str, int64_t ttl_days, rclcpp::Logger logger) : 
    ttl_days_(ttl_days), conn_str_(conn_str), logger_(logger) {}

void SensorSaverImpl::init_db() {
    db_conn_ = std::make_unique<pqxx::connection>(conn_str_);
    pqxx::work txn(*db_conn_);
    txn.exec(R"(
        CREATE TABLE IF NOT EXISTS mouse_movements (
            id SERIAL PRIMARY KEY,
        x REAL NOT NULL,
        y REAL NOT NULL,
        timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    ))");

    txn.commit();
    RCLCPP_INFO(logger_, "Database initialized successfully");
}

void SensorSaverImpl::cleanup_old_data() {
    pqxx::connection conn(conn_str_);
    pqxx::work txn(conn);
    std::string sql = "DELETE FROM mouseeeee_movements "
                          "WHERE timestamp < CURRENT_TIMESTAMP - INTERVAL '" +
                          std::to_string(ttl_days_) + " days'";

    auto result = txn.exec(sql);
    txn.commit();
    RCLCPP_INFO(logger_, "Cleaned %d old records", result.affected_rows());
}

void SensorSaverImpl::update_ttl(int64_t ttl_days_new) {
    ttl_days_ = ttl_days_new;
    RCLCPP_INFO(logger_, "TTL updated. TTL days: %ld", ttl_days_);
}

void SensorSaverImpl::save_to_db(double x, double y) {
    pqxx::connection conn(conn_str_);
    pqxx::work txn(conn);
    txn.exec_params("INSERT INTO mouse_movements (x, y) VALUES ($1, $2)", x, y);
    txn.commit();
    RCLCPP_INFO(logger_, "Saved coordinates: x=%f, y=%f", x, y);
}

void SensorSaverImpl::start_cleanup_timer(std::chrono::hours interval) {
    cleanup_active_ = true;
    cleanup_thread_ = std::thread([this, interval] {
        while (cleanup_active_) {
            std::this_thread::sleep_for(interval);
            cleanup_old_data();
        }
    });
    RCLCPP_INFO(logger_, "Cleanup timer started");
}

void SensorSaverImpl::stop_cleanup_timer() {
    cleanup_active_ = false;
    cleanup_thread_.join();
    RCLCPP_INFO(logger_, "Cleanup timer stopped");
}

void SensorSaverImpl::change_cleanup_interval(std::chrono::hours new_interval) {
    stop_cleanup_timer();
    start_cleanup_timer(new_interval);
    RCLCPP_INFO(logger_, "Cleanup interval changed to %ld hours", new_interval.count());
}
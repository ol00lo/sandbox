#include "sensor_saver_impl.hpp"

std::string DbConnectionSettings::build_connection_string() const {
    std::string res = "host=" + host +
                " dbname=" + dbname +
                " user=" + user +
                " password=" + password +
                " port=" + std::to_string(port);
    return res;
}

SensorSaverImpl::SensorSaverImpl(const DbConnectionSettings& settings, int64_t ttl_days, rclcpp::Logger logger,
                                 const std::string& table_name)
    : ttl_days_(ttl_days), logger_(logger), table_name_(table_name) {
    db_conn_ = std::make_unique<pqxx::connection>(settings.build_connection_string());
    pqxx::work txn(*db_conn_);

    std::string sql =
        "CREATE TABLE IF NOT EXISTS " + txn.quote_name(table_name_) + R"( (
        id SERIAL PRIMARY KEY,
        x REAL NOT NULL,
        y REAL NOT NULL,
        timestamp TIMESTAMP NOT NULL
    ))";
    txn.exec(sql);

    txn.commit();
    RCLCPP_INFO(logger_, "Database initialized successfully");
}

SensorSaverImpl::~SensorSaverImpl() {
    if (cleanup_active_) {
        stop_cleanup_timer();
    }
}

void SensorSaverImpl::cleanup_old_data() {
    pqxx::work txn(*db_conn_);
    std::string sql = "DELETE FROM " + txn.quote_name(table_name_) + " "
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

void SensorSaverImpl::save_to_db(double x, double y, std::chrono::system_clock::time_point timestamp) {
    const auto secs = std::chrono::time_point_cast<std::chrono::seconds>(timestamp);
    const auto epoch_secs = secs.time_since_epoch().count();

    pqxx::work txn(*db_conn_);
    txn.exec_params("INSERT INTO " + txn.quote_name(table_name_) +
                    " (x, y, timestamp) VALUES ($1, $2, to_timestamp($3))",
                    x, y, epoch_secs);
    txn.commit();
    RCLCPP_INFO(logger_, "Saved coordinates: x=%f, y=%f, timestamp=%ld", x, y, static_cast<long>(epoch_secs));
}

void SensorSaverImpl::start_cleanup_timer(std::chrono::hours interval) {
    cleanup_active_ = true;
    cleanup_thread_ = std::thread([this, interval] {
        if (interval.count() == 0) {
            cleanup_old_data();
            return;
        }
        while (cleanup_active_) {
            std::this_thread::sleep_for(interval);
            if (!cleanup_active_) break;
            cleanup_old_data();
        }
    });
    RCLCPP_INFO(logger_, "Cleanup timer started");
}

void SensorSaverImpl::stop_cleanup_timer() {
    cleanup_active_ = false;
    if (cleanup_thread_.joinable()) {
        cleanup_thread_.join();
    }
    RCLCPP_INFO(logger_, "Cleanup timer stopped");
}

void SensorSaverImpl::change_cleanup_interval(std::chrono::hours new_interval) {
    stop_cleanup_timer();
    start_cleanup_timer(new_interval);
    RCLCPP_INFO(logger_, "Cleanup interval changed to %ld hours", new_interval.count());
}

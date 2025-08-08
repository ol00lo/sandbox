#include "sensor_saver_impl.hpp"

SensorSaverImpl::SensorSaverImpl(const std::string& conn_str, int64_t ttl_days, Logger logger) : 
    ttl_days_(ttl_days), conn_str_(conn_str), logger_(logger) {}

void SensorSaverImpl::init_db() {
    try{
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
        logger_("INFO", "Database initialized successfully");
    } catch (const std::exception& e) {
        logger_("ERROR", "Failed to initialize database: " + std::string(e.what()));
        throw;
    }

}

void SensorSaverImpl::cleanup_old_data() {
    try{
        pqxx::connection conn(conn_str_);
        pqxx::work txn(conn);
        std::string sql = "DELETE FROM mouse_movements "
                              "WHERE timestamp < CURRENT_TIMESTAMP - INTERVAL '" +
                              std::to_string(ttl_days_) + " days'";

        auto result = txn.exec(sql);
        txn.commit();
        logger_("INFO", "Cleaned " + std::to_string(result.affected_rows()) + " old records");
    } catch (const std::exception& e) {
        logger_("ERROR", std::string("Cleanup failed: ") + e.what());
        throw;
    }
}

void SensorSaverImpl::update_ttl(int64_t ttl_days_new) {
    ttl_days_ = ttl_days_new;
    logger_("INFO", "TTL updated. TTL days: " + std::to_string(ttl_days_));
}

void SensorSaverImpl::save_to_db(double x, double y) {
    try{
        pqxx::connection conn(conn_str_);
        pqxx::work txn(conn);
        txn.exec_params("INSERT INTO mouse_movements (x, y) VALUES ($1, $2)", x, y);
        txn.commit();
        logger_("INFO", "Saved coordinates: x=" + std::to_string(x) + 
        ", y=" + std::to_string(y));
    } catch (const std::exception& e) {
        logger_("ERROR", std::string("Save failed: ") + e.what());
        throw;
    }
}

void SensorSaverImpl::start_cleanup_timer(std::chrono::hours interval) {
    cleanup_active_ = true;
    cleanup_thread_ = std::thread([this, interval] {
        while (cleanup_active_) {
            std::this_thread::sleep_for(interval);
            cleanup_old_data();
        }
    });
    logger_("INFO", "Cleanup timer started");
}

void SensorSaverImpl::stop_cleanup_timer() {
    cleanup_active_ = false;
    cleanup_thread_.join();
    logger_("INFO", "Cleanup timer stopped");
}

void SensorSaverImpl::change_cleanup_interval(std::chrono::hours new_interval) {
    stop_cleanup_timer();
    start_cleanup_timer(new_interval);
    logger_("INFO", "Cleanup interval changed to " + std::to_string(new_interval.count()) + " hours");
}
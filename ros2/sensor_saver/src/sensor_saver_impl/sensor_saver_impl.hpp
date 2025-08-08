#ifndef SENSOR_SAVER_IMPL_HPP
#define SENSOR_SAVER_IMPL_HPP

#include <pqxx/pqxx>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>

class SensorSaverImpl {
public:
    using Logger = std::function<void(const std::string&, const std::string&)>;
    SensorSaverImpl(const std::string& conn_str, int64_t ttl_days, Logger logger);

    void init_db();
    void cleanup_old_data();
    void update_ttl(int64_t ttl_days_new);
    void save_to_db(double x, double y);

    void start_cleanup_timer(std::chrono::hours time);
    void change_cleanup_interval(std::chrono::hours new_interval);
    void stop_cleanup_timer();

private:
    std::unique_ptr<pqxx::connection> db_conn_;
    int64_t ttl_days_;
    std::string conn_str_;
    Logger logger_;
    std::atomic<bool> cleanup_active_{false};
    std::thread cleanup_thread_;
};


#endif
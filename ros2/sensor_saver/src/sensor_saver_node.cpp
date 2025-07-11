#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <pqxx/pqxx>
#include <rclcpp/rclcpp.hpp>

class MouseSaver : public rclcpp::Node
{
public:
    MouseSaver() : Node("MouseSaver") {
        this->declare_parameter<int64_t>("sensor_data_ttl_days", 7);
        this->declare_parameter<std::string>("db_connection.host", "db");
        this->declare_parameter<std::string>("db_connection.dbname", "mydb");
        this->declare_parameter<std::string>("db_connection.user", "user");
        this->declare_parameter<std::string>("db_connection.password", "password");
        this->declare_parameter<int>("db_connection.port", 5432);

        ttl_days_ = this->get_parameter("sensor_data_ttl_days").as_int();
        conn_str_ = "host=" + this->get_parameter("db_connection.host").as_string() +
                    " dbname=" + this->get_parameter("db_connection.dbname").as_string() +
                    " user=" + this->get_parameter("db_connection.user").as_string() +
                    " password=" + this->get_parameter("db_connection.password").as_string() +
                    " port=" + std::to_string(this->get_parameter("db_connection.port").as_int());

        RCLCPP_INFO(this->get_logger(), "TTL days: %ld", ttl_days_);
        RCLCPP_INFO(this->get_logger(), "DB connection: %s", conn_str_.substr(0, conn_str_.find("password")).c_str());

        try {
            db_conn_ = std::make_unique<pqxx::connection>(conn_str_);
            init_db();
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "DB connection failed: %s", e.what());
            throw;
        }

        subscription_ = create_subscription<geometry_msgs::msg::Point>(
            "mouse_moved", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg) { save_to_db(msg->x, msg->y); });

        cleanup_timer_ = this->create_wall_timer(std::chrono::hours(24), std::bind(&MouseSaver::cleanup_old_data, this));

        RCLCPP_INFO(get_logger(), "Subscriber ready. DB: %s", conn_str_.substr(0, conn_str_.find("password")).c_str());
    }

private:
    void init_db() {
        try {
            pqxx::work txn(*db_conn_);

            txn.exec(R"(
                CREATE TABLE IF NOT EXISTS mouse_movements (
                    id SERIAL PRIMARY KEY,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                ))");

            txn.commit();
            RCLCPP_INFO(get_logger(), "Table initialized");
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "DB init failed: %s", e.what());
            throw;
        }
    }

    void save_to_db(double x, double y) {
        try {
            pqxx::connection conn(conn_str_);
            pqxx::work txn(conn);

            txn.exec_params("INSERT INTO mouse_movements (x, y) VALUES ($1, $2)", x, y);

            txn.commit();
            RCLCPP_INFO(get_logger(), "Saved to DB: x=%.2f, y=%.2f", x, y);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "DB error: %s", e.what());
        }
    }

    void cleanup_old_data() {
        try {
            pqxx::connection conn(conn_str_);
            pqxx::work txn(conn);

            std::string sql = "DELETE FROM mouse_movements "
                              "WHERE timestamp < CURRENT_TIMESTAMP - INTERVAL '" +
                              std::to_string(ttl_days_) + " days'";

            auto result = txn.exec(sql);
            txn.commit();

            RCLCPP_INFO(get_logger(), "Cleaned up %d old records", result.affected_rows());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Cleanup failed: %s", e.what());
        }
    }

    int64_t ttl_days_;
    std::string conn_str_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr cleanup_timer_;
    std::unique_ptr<pqxx::connection> db_conn_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MouseSaver>());
    rclcpp::shutdown();
    return 0;
}
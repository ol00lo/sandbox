#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pqxx/pqxx>
#include <memory>

class MouseSaver : public rclcpp::Node {
public:
    MouseSaver() : Node("MouseSaver") {
		this->declare_parameter<int>("sensor_data_ttl_days", 7);
		this->declare_parameter<std::string>("db_connection",
			"host=db dbname=mydb user=user password=password port=5432");

		ttl_days_ = this->get_parameter("sensor_data_ttl_days").as_int();
        conn_str_ = this->get_parameter("db_connection").as_string();

		RCLCPP_INFO(this->get_logger(), "TTL days: %d", ttl_days_);
        RCLCPP_INFO(this->get_logger(), "DB connection: %s",
                   conn_str_.substr(0, conn_str_.find("password")).c_str());

        init_db();

        subscription_ = create_subscription<geometry_msgs::msg::Point>(
            "mouse_moved", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                save_to_db(msg->x, msg->y);
            });

        cleanup_timer_ = this->create_wall_timer(
            std::chrono::hours(24),
            [this]() { cleanup_old_data(); });

        RCLCPP_INFO(get_logger(), "Subscriber ready. DB: %s", 
                   conn_str_.substr(0, conn_str_.find("password")).c_str());
    }

private:
    void init_db() {
        try {
            pqxx::connection conn(conn_str_);
            pqxx::work txn(conn);

            txn.exec(R"(
                CREATE TABLE IF NOT EXISTS mouse_movements (
                    id SERIAL PRIMARY KEY,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                ))");

            txn.commit();
            RCLCPP_INFO(get_logger(), "Table initialized");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "DB init failed: %s", e.what());
        }
    }

    void save_to_db(float x, float y) {
        try {
            pqxx::connection conn(conn_str_);
            pqxx::work txn(conn);

            txn.exec_params(
                "INSERT INTO mouse_movements (x, y) VALUES ($1, $2)",
                x, y);

            txn.commit();
            RCLCPP_INFO(get_logger(), "Saved to DB: x=%.2f, y=%.2f", x, y);
        } catch (const std::exception &e) {
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
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Cleanup failed: %s", e.what());
        }
    }

    int ttl_days_;
    std::string conn_str_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr cleanup_timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MouseSaver>());
    rclcpp::shutdown();
    return 0;
}
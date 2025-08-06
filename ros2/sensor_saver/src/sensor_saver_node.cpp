#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include "sensor_saver_impl.hpp"

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
        std::string conn_str = "host=" + this->get_parameter("db_connection.host").as_string() +
                    " dbname=" + this->get_parameter("db_connection.dbname").as_string() +
                    " user=" + this->get_parameter("db_connection.user").as_string() +
                    " password=" + this->get_parameter("db_connection.password").as_string() +
                    " port=" + std::to_string(this->get_parameter("db_connection.port").as_int());

        auto logger = [this](const std::string& level, const std::string& msg) {
                if (level == "ERROR") RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
                else RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        };

        db_ = std::make_unique<SensorSaverImpl>(conn_str, ttl_days_, logger);
        db_->init_db();

        RCLCPP_INFO(this->get_logger(), "TTL days: %ld", ttl_days_);
        RCLCPP_INFO(this->get_logger(), "DB connection: %s", conn_str.substr(0, conn_str.find("password")).c_str());

        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&MouseSaver::parameters_callback, this, std::placeholders::_1));

        subscription_ = create_subscription<geometry_msgs::msg::Point>(
            "mouse_moved", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg) { db_->save_to_db(msg->x, msg->y); });

        cleanup_timer_ = this->create_wall_timer(std::chrono::hours(2), [this]() { db_->cleanup_old_data(); });

        RCLCPP_INFO(get_logger(), "Subscriber ready. DB: %s", conn_str.substr(0, conn_str.find("password")).c_str());
    }

private:
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "sensor_data_ttl_days") {
                ttl_days_ = param.as_int();
                RCLCPP_INFO(get_logger(), "TTL days updated to: %ld", ttl_days_);
            } else {
                result.successful = false;
                result.reason = "Parameter '" + param.get_name() + "' cannot be changed at runtime";
                RCLCPP_ERROR(get_logger(), "Attempt to change read-only parameter: %s", 
                            param.get_name().c_str());
            }
        }

        return result;
    }

    int64_t ttl_days_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr cleanup_timer_;
    std::unique_ptr<SensorSaverImpl> db_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MouseSaver>());
    rclcpp::shutdown();
    return 0;
}

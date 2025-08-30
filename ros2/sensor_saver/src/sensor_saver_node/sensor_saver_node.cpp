#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include "sensor_saver_impl.hpp"

constexpr int64_t SENSOR_DATA_TTL_DAYS_DEFAULT = 7;
constexpr const char* HOST_DEFAULT = "db";
constexpr const char* DBNAME_DEFAULT = "mydb";
constexpr const char* USER_DEFAULT = "user";
constexpr const char* PASSWORD_DEFAULT = "password";
constexpr int PORT_DEFAULT = 5432;
constexpr int CLEANUP_INTERVAL_HOURS_DEFAULT = 24;

class MouseSaver : public rclcpp::Node {
public:
    MouseSaver() : Node("MouseSaver") {
        this->declare_parameter<int64_t>("sensor_data_ttl_days", SENSOR_DATA_TTL_DAYS_DEFAULT);
        this->declare_parameter<std::string>("db_connection.host", HOST_DEFAULT);
        this->declare_parameter<std::string>("db_connection.dbname", DBNAME_DEFAULT);
        this->declare_parameter<std::string>("db_connection.user", USER_DEFAULT);
        this->declare_parameter<std::string>("db_connection.password", PASSWORD_DEFAULT);
        this->declare_parameter<int>("db_connection.port", PORT_DEFAULT);
        this->declare_parameter<int>("cleanup_interval_hours", CLEANUP_INTERVAL_HOURS_DEFAULT);

        int64_t ttl_days = this->get_parameter("sensor_data_ttl_days").as_int();
        DbConnectionSettings settings = DbConnectionSettings{
                this->get_parameter("db_connection.host").as_string(),
                this->get_parameter("db_connection.dbname").as_string(),
                this->get_parameter("db_connection.user").as_string(),
                this->get_parameter("db_connection.password").as_string(),
                static_cast<int>(this->get_parameter("db_connection.port").as_int())};

        try {
            pimpl_ = std::make_unique<SensorSaverImpl>(settings, ttl_days, this->get_logger());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize database: %s", e.what());
            throw;
        }
        pimpl_->start_cleanup_timer(std::chrono::hours(CLEANUP_INTERVAL_HOURS_DEFAULT));
        rclcpp::on_shutdown([this]() { pimpl_->stop_cleanup_timer(); });

        RCLCPP_INFO(this->get_logger(), "TTL days: %ld", ttl_days);
        RCLCPP_INFO(this->get_logger(), "DB connection: host = %s, dbname = %s, user = %s", 
                   settings.host.c_str(), settings.dbname.c_str(), settings.user.c_str());

        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&MouseSaver::parameters_callback, this, std::placeholders::_1));

        subscription_ = create_subscription<geometry_msgs::msg::Point>(
            "mouse_moved", 10,
            [this](geometry_msgs::msg::Point::SharedPtr msg, const rclcpp::MessageInfo & info) {
                auto ns = info.get_rmw_message_info().source_timestamp;
                auto tp = std::chrono::system_clock::time_point(std::chrono::nanoseconds(ns));
                pimpl_->save_to_db(msg->x, msg->y, tp);
            });

        RCLCPP_INFO(get_logger(), "Subscriber ready. DB: host = %s, dbname = %s, user = %s", 
                   settings.host.c_str(), settings.dbname.c_str(), settings.user.c_str());
    }

private:
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name() == "sensor_data_ttl_days") {
                int64_t ttl_days = param.as_int();
                pimpl_->update_ttl(ttl_days);
                RCLCPP_INFO(get_logger(), "TTL days updated to: %ld", ttl_days);
            }
            else if (param.get_name() == "cleanup_interval_hours") {
                int64_t hours = param.as_int();
                if (hours <= 0) {
                    RCLCPP_ERROR(get_logger(), "Parameter '%s' must be greater than 0: ", param.get_name().c_str());
                }
                pimpl_->change_cleanup_interval(std::chrono::hours(hours));
            }

            else {
                result.successful = false;
                result.reason = "Parameter '" + param.get_name() + "' cannot be changed at runtime";
                RCLCPP_ERROR(get_logger(), "Attempt to change read-only parameter: %s", param.get_name().c_str());
            }
        }

        return result;
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    std::unique_ptr<SensorSaverImpl> pimpl_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MouseSaver>());
    rclcpp::shutdown();
    return 0;
}

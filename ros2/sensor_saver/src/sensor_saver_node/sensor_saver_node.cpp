#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include "sensor_saver_impl.hpp"

class MouseSaver : public rclcpp::Node
{
public:
    MouseSaver() : Node("MouseSaver") {
        this->declare_parameter<int64_t>("sensor_data_ttl_days", sensor_data_ttl_days_default_);
        this->declare_parameter<std::string>("db_connection.host", host_default_);
        this->declare_parameter<std::string>("db_connection.dbname", dbname_default_);
        this->declare_parameter<std::string>("db_connection.user", user_default_);
        this->declare_parameter<std::string>("db_connection.password", password_default_);
        this->declare_parameter<int>("db_connection.port", port_default_);
        this->declare_parameter<int64_t>("cleanup_interval_hours", cleanup_interval_default_);

        int64_t ttl_days = this->get_parameter("sensor_data_ttl_days").as_int();
        std::string conn_str = "host=" + this->get_parameter("db_connection.host").as_string() +
                    " dbname=" + this->get_parameter("db_connection.dbname").as_string() +
                    " user=" + this->get_parameter("db_connection.user").as_string() +
                    " password=" + this->get_parameter("db_connection.password").as_string() +
                    " port=" + std::to_string(this->get_parameter("db_connection.port").as_int());

        auto logger = [this](const std::string& level, const std::string& msg) {
                if (level == "ERROR") RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
                else RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        };

        pimpl_ = std::make_unique<SensorSaverImpl>(conn_str, ttl_days, logger);
        pimpl_->init_db();
        pimpl_->start_cleanup_timer(std::chrono::hours(cleanup_interval_default_));
        rclcpp::on_shutdown([this]() {pimpl_->stop_cleanup_timer();});

        RCLCPP_INFO(this->get_logger(), "TTL days: %ld", ttl_days);
        RCLCPP_INFO(this->get_logger(), "DB connection: %s", conn_str.substr(0, conn_str.find("password")).c_str());

        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&MouseSaver::parameters_callback, this, std::placeholders::_1));

        subscription_ = create_subscription<geometry_msgs::msg::Point>(
            "mouse_moved", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg) { pimpl_->save_to_db(msg->x, msg->y); });


        RCLCPP_INFO(get_logger(), "Subscriber ready. DB: %s", conn_str.substr(0, conn_str.find("password")).c_str());
    }

private:
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
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
                RCLCPP_ERROR(get_logger(), "Attempt to change read-only parameter: %s", 
                            param.get_name().c_str());
            }
        }

        return result;
    }

    const int64_t sensor_data_ttl_days_default_ = 7;
    const std::string host_default_ = "db";
    const std::string dbname_default_ = "mydb";
    const std::string user_default_ = "user";
    const std::string password_default_ = "password";
    const int port_default_ = 5432;
    const int64_t cleanup_interval_default_ = 2;

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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interfaces/msg/obstacle_info.hpp"
#include <custom_interfaces/srv/set_threshold.hpp>
#include <cmath>
#include <memory>
#include <limits>
#include <string>

class SafetyController : public rclcpp::Node
{
public:
    SafetyController() : Node("safety_controller")
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&SafetyController::scan_callback, this, std::placeholders::_1));
        obstacle_publisher_ = this->create_publisher<custom_interfaces::msg::ObstacleInfo>("/obstacle_info", 10);
        threshold_service_ = this->create_service<custom_interfaces::srv::SetThreshold>("/set_threshold", std::bind(&SafetyController::set_threshold_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Safety controller started. Listening to /scan topic.");
    }
private:
    void set_threshold_callback(const std::shared_ptr<custom_interfaces::srv::SetThreshold::Request> request,
                                std::shared_ptr<custom_interfaces::srv::SetThreshold::Response> response)
    {
        threshold_ = request->threshold;
        RCLCPP_INFO(this->get_logger(), "Threshold updated to: %.2f", threshold_);
        response->success = true;
        response->message = "Threshold updated successfully.";
    }
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double minimum_distance = std::numeric_limits<double>::infinity();
        int minimum_index = -1;
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double distance = msg->ranges[i];
            if(!std::isfinite(distance))
            {
                continue; // Skip NaN and Inf values
            }
            if (distance < msg->range_min || distance > msg->range_max)
            {
                continue; // Skip values outside the valid range
            }
            if (distance < minimum_distance)
            {
                minimum_distance = distance;
                minimum_index = i;
            }
        }
        if (minimum_index == -1)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No valid obstacle detected in the scan data.");
            return;
        }
        double angle = msg->angle_min + minimum_index * msg->angle_increment;
        std::string direction;
        const double pi = 3.14159265358979323846;
        if (angle > -pi / 4.0 && angle < pi / 4.0)
        {
            direction = "front";
        }
        else if (angle >= pi / 4.0 && angle < 3.0 * pi / 4.0)
        {
            direction = "left";
        }
        else if (angle <= -pi / 4.0 && angle > -3 * pi / 4.0)
        {
            direction = "right";
        }
        else
        {
            direction = "back";
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Closest obstacle detected at %.2f m | Angle: %.2f deg | Direction: %s", minimum_distance, angle * 180.0 / pi, direction.c_str());
        custom_interfaces::msg::ObstacleInfo obstacle_msg;
        obstacle_msg.distance = minimum_distance;
        obstacle_msg.direction = direction;
        obstacle_msg.threshold = threshold_;
        obstacle_publisher_->publish(obstacle_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<custom_interfaces::msg::ObstacleInfo>::SharedPtr obstacle_publisher_;
    rclcpp::Service<custom_interfaces::srv::SetThreshold>::SharedPtr threshold_service_;
    double threshold_ = 2.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
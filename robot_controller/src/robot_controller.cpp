#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_interfaces/msg/obstacle_info.hpp"
#include "custom_interfaces/srv/get_average_velocity.hpp"

#include <iostream>
#include <memory>
#include <thread>
#include <limits>
#include <deque>

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller"), current_linear_velocity_(0.0), current_angular_velocity_(0.0), running_(true)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        obstacle_subscriber_ = this->create_subscription<custom_interfaces::msg::ObstacleInfo>("obstacle_info", 10, std::bind(&RobotController::obstacle_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RobotController::publish_velocity, this));
        input_thread_ = std::thread(&RobotController::get_user_input, this);
        avg_velocity_service_ = this->create_service<custom_interfaces::srv::GetAverageVelocity>("/get_average_velocity", std::bind(&RobotController::get_average_velocity_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    ~RobotController()
    {
        running_ = false;
        if (input_thread_.joinable())
        {
            input_thread_.join();
        }
    }
private:
    void get_average_velocity_callback(const std::shared_ptr<custom_interfaces::srv::GetAverageVelocity::Request> request,
                                       std::shared_ptr<custom_interfaces::srv::GetAverageVelocity::Response> response)
    {
        (void)request; // Unused parameter
        if (velocity_history_.empty())
        {
            response->average_linear = 0.0;
            response->average_angular = 0.0;
            return;
        }

        double sum_linear = 0.0;
        double sum_angular = 0.0;
        for (const auto& entry : velocity_history_)
        {
            sum_linear += entry.linear;
            sum_angular += entry.angular;
        }
        response->average_linear = sum_linear / velocity_history_.size();
        response->average_angular = sum_angular / velocity_history_.size();
    }
    void obstacle_callback(const custom_interfaces::msg::ObstacleInfo::SharedPtr msg)
    {
        if (msg->distance < msg->threshold && !recovering_ && !stop_after_recovery_ && current_linear_velocity_ != 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected too close! Robot will step back.");
            recovering_ = true;
        }
    }
    void publish_velocity()
    {
        geometry_msgs::msg::Twist message;
        if (recovering_)
        {
            message.linear.x = -current_linear_velocity_; 
            message.angular.z = 0.0; 
            recovering_ = false;
            stop_after_recovery_ = true;
            RCLCPP_WARN(this->get_logger(), "Robot stepped back and stopped.");
        }
        else if (stop_after_recovery_)
        {
            message.linear.x = 0.0;
            message.angular.z = 0.0; 
        }
        else
        {
            message.linear.x = current_linear_velocity_;
            message.angular.z = current_angular_velocity_;
        }
        publisher_->publish(message);
    }
    void get_user_input()
    {
        while (running_)
        {
            double linear_velocity, angular_velocity;
            std::cout << "Enter [linear angular] velocities: ";
            if(!(std::cin >> linear_velocity >> angular_velocity)) {
                std::cerr << "Invalid input. Please enter two numbers." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }
            current_linear_velocity_ = linear_velocity;
            current_angular_velocity_ = angular_velocity;
            if (linear_velocity != 0.0 || angular_velocity != 0.0) {
                stop_after_recovery_ = false;
            }
            VelocityEntry input;
            input.linear = linear_velocity;
            input.angular = angular_velocity;
            velocity_history_.push_back(input);
            if (velocity_history_.size() > 5) {
                velocity_history_.pop_front();
            }
            std::cout << "Updated velocities: linear = " << current_linear_velocity_ << ", angular = " << current_angular_velocity_ << std::endl;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<custom_interfaces::msg::ObstacleInfo>::SharedPtr obstacle_subscriber_;
    rclcpp::Service<custom_interfaces::srv::GetAverageVelocity>::SharedPtr avg_velocity_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;
    double current_linear_velocity_;
    double current_angular_velocity_;
    bool running_;
    bool recovering_ = false;
    bool stop_after_recovery_ = false;
    struct VelocityEntry
    {
        double linear;
        double angular;
    };
    std::deque<VelocityEntry> velocity_history_;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
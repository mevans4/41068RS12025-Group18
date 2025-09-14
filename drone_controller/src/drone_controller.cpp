#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DroneController : public rclcpp::Node
{
public:
    DroneController() : Node("drone_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&DroneController::publish_command, this));

        RCLCPP_INFO(this->get_logger(), "Drone controller node started.");
    }

private:
    void publish_command()
    {
        auto msg = geometry_msgs::msg::Twist();

        // Hover command: Apply slight upward force to maintain altitude
        msg.linear.x = 0.3;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;  // Adjust this value depending on gravity & tuning

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}

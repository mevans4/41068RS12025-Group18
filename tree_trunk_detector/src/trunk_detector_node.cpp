#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tree_trunk_detector/trunk_detector.hpp"

class TrunkDetectorNode : public rclcpp::Node {
public:
    TrunkDetectorNode() : Node("trunk_detector_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&TrunkDetectorNode::scanCallback, this, std::placeholders::_1));
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto result = detector_.detectTrunk(msg->ranges, msg->angle_min, msg->angle_increment);

        if (result.valid) {
            RCLCPP_INFO(this->get_logger(), "Detected trunk width: %.2f m", result.width_m);
        } else {
            RCLCPP_WARN(this->get_logger(), "No trunk detected.");
        }
    }

    TrunkDetector detector_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrunkDetectorNode>());
    rclcpp::shutdown();
    return 0;
}

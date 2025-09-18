#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <cmath>

class LidarTreeDetectorNode : public rclcpp::Node
{
public:
    LidarTreeDetectorNode() : Node("lidar_tree_detector")
    {
        RCLCPP_INFO(this->get_logger(), "LidarTreeDetectorNode has started.");
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LidarTreeDetectorNode::lidar_callback, this, std::placeholders::_1)
        );
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received LaserScan message with %zu ranges", msg->ranges.size());

        // Step 1: Preprocess data
        std::vector<std::pair<float, float>> points = preprocess_lidar(msg);
        RCLCPP_DEBUG(this->get_logger(), "Preprocessed %zu points", points.size());

        // Step 2: Detect trees (placeholder logic)
        std::vector<std::pair<float, float>> tree_positions = detect_trees(points);
        RCLCPP_INFO(this->get_logger(), "Detected %zu trees", tree_positions.size());

        // Step 3: Analyze trees (placeholder logic)
        analyze_trees(tree_positions);
    }

    // Convert polar coordinates to Cartesian (x, y)
    std::vector<std::pair<float, float>> preprocess_lidar(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
    {
        std::vector<std::pair<float, float>> points;
        float angle = msg->angle_min;
        for (const auto& range : msg->ranges)
        {
            if (std::isfinite(range))
            {
                float x = range * std::cos(angle);
                float y = range * std::sin(angle);
                points.emplace_back(x, y);
            }
            angle += msg->angle_increment;
        }
        return points;
    }

    // Placeholder: Detect trees (clusters) in the point cloud
    std::vector<std::pair<float, float>> detect_trees(const std::vector<std::pair<float, float>>& points)
    {
        std::vector<std::pair<float, float>> tree_positions;
        // TODO: Replace with real clustering/tree detection logic
        if (!points.empty())
        {
            // For demonstration, just pick every 50th point as a "tree"
            for (size_t i = 0; i < points.size(); i += 50)
            {
                tree_positions.push_back(points[i]);
                RCLCPP_DEBUG(this->get_logger(), "Tree candidate at (%.2f, %.2f)", points[i].first, points[i].second);
            }
        }
        return tree_positions;
    }

    // Placeholder: Analyze detected trees
    void analyze_trees(const std::vector<std::pair<float, float>>& tree_positions)
    {
        for (size_t i = 0; i < tree_positions.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Tree %zu at (x=%.2f, y=%.2f)", i, tree_positions[i].first, tree_positions[i].second);
            // TODO: Add more analysis (e.g., estimate diameter, height, etc.)
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // Set logger level to DEBUG for detailed output
    rclcpp::Logger logger = rclcpp::get_logger("rclcpp");
    if (rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
        RCLCPP_WARN(logger, "Failed to set logger level to DEBUG");
    }
    rclcpp::spin(std::make_shared<LidarTreeDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
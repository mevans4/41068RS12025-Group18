#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <cmath>
#include <sstream>

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
        publisher_ = this->create_publisher<std_msgs::msg::String>("detected_trees", 10);
    }

private:
    // Store unique tree positions
    std::vector<std::pair<float, float>> unique_trees_;
    const float duplicate_threshold_ = 1.0; // meters

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<std::pair<float, float>> points = preprocess_lidar(msg);
        std::vector<std::pair<float, float>> tree_positions = detect_trees(points);

        for (const auto& tree : tree_positions)
        {
            if (!is_duplicate(tree)) {
                unique_trees_.push_back(tree);
                RCLCPP_INFO(this->get_logger(), "Added new tree at (%.2f, %.2f)", tree.first, tree.second);
            }
        }

        publish_tree_array();
    }

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

    std::vector<std::pair<float, float>> detect_trees(const std::vector<std::pair<float, float>>& points)
{
    std::vector<std::pair<float, float>> tree_positions;
    if (points.empty()) return tree_positions;

    const float cluster_dist_threshold = 0.5; // meters, adjust as needed
    const size_t min_cluster_size = 3;        // minimum points to consider a tree

    std::vector<std::pair<float, float>> cluster;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (cluster.empty())
        {
            cluster.push_back(points[i]);
        }
        else
        {
            float dx = points[i].first - cluster.back().first;
            float dy = points[i].second - cluster.back().second;
            float dist = std::sqrt(dx*dx + dy*dy);
            if (dist < cluster_dist_threshold)
            {
                cluster.push_back(points[i]);
            }
            else
            {
                // End of cluster
                if (cluster.size() >= min_cluster_size)
                {
                    // Compute centroid
                    float sum_x = 0, sum_y = 0;
                    for (const auto& p : cluster)
                    {
                        sum_x += p.first;
                        sum_y += p.second;
                    }
                    tree_positions.emplace_back(sum_x / cluster.size(), sum_y / cluster.size());
                }
                cluster.clear();
                cluster.push_back(points[i]);
            }
        }
    }
    // Check last cluster
    if (cluster.size() >= min_cluster_size)
    {
        float sum_x = 0, sum_y = 0;
        for (const auto& p : cluster)
        {
            sum_x += p.first;
            sum_y += p.second;
        }
        tree_positions.emplace_back(sum_x / cluster.size(), sum_y / cluster.size());
    }
    return tree_positions;
}

    bool is_duplicate(const std::pair<float, float>& new_tree)
    {
        for (const auto& tree : unique_trees_)
        {
            float dx = tree.first - new_tree.first;
            float dy = tree.second - new_tree.second;
            float dist = std::sqrt(dx*dx + dy*dy);
            if (dist < duplicate_threshold_)
                return true;
        }
        return false;
    }

    void publish_tree_array()
    {
        std_msgs::msg::String msg;
        std::ostringstream oss;
        oss << "Detected trees (" << unique_trees_.size() << "): ";
        for (size_t i = 0; i < unique_trees_.size(); ++i)
        {
            oss << "[" << unique_trees_[i].first << ", " << unique_trees_[i].second << "]";
            if (i != unique_trees_.size() - 1)
                oss << ", ";
        }
        msg.data = oss.str();
        publisher_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarTreeDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
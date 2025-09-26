#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <cmath>
#include <sstream>

struct Tree
{
    int id;
    float x_avg;
    float y_avg;
    std::vector<float> diameters;
    int count;

    Tree(int id_, float x, float y, float diameter)
        : id(id_), x_avg(x), y_avg(y), diameters{diameter}, count(1) {}

    float latest_diameter() const { return diameters.empty() ? 0.0f : diameters.back(); }
    float average_diameter() const {
        float sum = 0;
        for (float d : diameters) sum += d;
        return diameters.empty() ? 0.0f : sum / diameters.size();
    }
};

class LidarTreeDetectorNode : public rclcpp::Node
{
public:
    LidarTreeDetectorNode() : Node("lidar_tree_detector"), next_tree_id_(1)
    {
        RCLCPP_INFO(this->get_logger(), "LidarTreeDetectorNode has started.");
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LidarTreeDetectorNode::lidar_callback, this, std::placeholders::_1)
        );
        publisher_ = this->create_publisher<std_msgs::msg::String>("detected_trees", 10);
    }

private:
    std::vector<Tree> trees_;
    int next_tree_id_;
    const float match_threshold_ = 1.0; // meters

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<std::pair<float, float>> points = preprocess_lidar(msg);
        auto clusters = cluster_points(points);

        for (const auto& cluster : clusters)
        {
            auto [cx, cy] = compute_centroid(cluster);
            float diameter = estimate_diameter(cluster);

            int match_idx = find_matching_tree(cx, cy);
            if (match_idx >= 0) {
                // Update running averages and diameter history
                Tree& t = trees_[match_idx];
                t.count++;
                t.x_avg += (cx - t.x_avg) / t.count;
                t.y_avg += (cy - t.y_avg) / t.count;
                t.diameters.push_back(diameter);
            } else {
                trees_.emplace_back(next_tree_id_++, cx, cy, diameter);
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

    std::vector<std::vector<std::pair<float, float>>> cluster_points(const std::vector<std::pair<float, float>>& points)
    {
        std::vector<std::vector<std::pair<float, float>>> clusters;
        if (points.empty()) return clusters;

        const float cluster_dist_threshold = 0.5; // meters
        const size_t min_cluster_size = 3;

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
                    if (cluster.size() >= min_cluster_size)
                        clusters.push_back(cluster);
                    cluster.clear();
                    cluster.push_back(points[i]);
                }
            }
        }
        if (cluster.size() >= min_cluster_size)
            clusters.push_back(cluster);

        return clusters;
    }

    std::pair<float, float> compute_centroid(const std::vector<std::pair<float, float>>& cluster)
    {
        float sum_x = 0, sum_y = 0;
        for (const auto& p : cluster)
        {
            sum_x += p.first;
            sum_y += p.second;
        }
        return {sum_x / cluster.size(), sum_y / cluster.size()};
    }

    float estimate_diameter(const std::vector<std::pair<float, float>>& cluster)
    {
        float max_dist = 0;
        for (size_t i = 0; i < cluster.size(); ++i)
        {
            for (size_t j = i + 1; j < cluster.size(); ++j)
            {
                float dx = cluster[i].first - cluster[j].first;
                float dy = cluster[i].second - cluster[j].second;
                float dist = std::sqrt(dx*dx + dy*dy);
                if (dist > max_dist)
                    max_dist = dist;
            }
        }
        return max_dist;
    }

    int find_matching_tree(float x, float y)
    {
        for (size_t i = 0; i < trees_.size(); ++i)
        {
            float dx = trees_[i].x_avg - x;
            float dy = trees_[i].y_avg - y;
            float dist = std::sqrt(dx*dx + dy*dy);
            if (dist < match_threshold_)
                return i;
        }
        return -1;
    }

    void publish_tree_array()
    {
        std_msgs::msg::String msg;
        std::ostringstream oss;
        oss << "Detected trees (" << trees_.size() << "): ";
        for (size_t i = 0; i < trees_.size(); ++i)
        {
            oss << "[id=" << trees_[i].id
                << ", x=" << trees_[i].x_avg
                << ", y=" << trees_[i].y_avg
                << ", avg_diam=" << trees_[i].average_diameter()
                << ", latest_diam=" << trees_[i].latest_diameter()
                << ", obs=" << trees_[i].count << "]";
            if (i != trees_.size() - 1)
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
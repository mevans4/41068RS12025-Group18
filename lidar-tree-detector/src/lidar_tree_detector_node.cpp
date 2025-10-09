#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <limits>

struct Tree
{
    int id;
    double x_avg;
    double y_avg;
    std::vector<double> diameters;
    int count;

    Tree(int id_, double x, double y, double diameter)
        : id(id_), x_avg(x), y_avg(y), diameters{diameter}, count(1) {}

    double latest_diameter() const { return diameters.empty() ? 0.0 : diameters.back(); }
    double average_diameter() const {
        double sum = 0.0;
        for (double d : diameters) sum += d;
        return diameters.empty() ? 0.0 : sum / diameters.size();
    }
};

class LidarTreeDetectorNode : public rclcpp::Node
{
public:
    LidarTreeDetectorNode()
    : Node("lidar_tree_detector"), next_tree_id_(1)
    {
        // declare tunable parameters
        declare_parameter<std::string>("scan_topic", "/scan");
        declare_parameter<std::string>("detected_topic", "/detected_trees");
        declare_parameter<double>("max_distance", 15.0);
        declare_parameter<double>("cluster_dist_threshold", 0.6);
        declare_parameter<int>("min_cluster_size", 5);
        declare_parameter<double>("match_threshold", 1.0);
        declare_parameter<double>("curvature_ratio_threshold", 0.03);
        declare_parameter<double>("curvature_abs_threshold", 0.03);
        declare_parameter<double>("fit_max_rmse", 0.08);
        declare_parameter<double>("fit_min_radius", 0.02);
        declare_parameter<double>("fit_max_radius", 1.2);
        declare_parameter<bool>("accept_fallback", true);
        declare_parameter<bool>("debug_reasons", false);

        load_parameters();

        RCLCPP_INFO(get_logger(), "Started lidar_tree_detector (scan=%s detected=%s)",
                    scan_topic_.c_str(), detected_topic_.c_str());

        subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10,
            std::bind(&LidarTreeDetectorNode::lidar_callback, this, std::placeholders::_1)
        );
        publisher_ = create_publisher<std_msgs::msg::String>(detected_topic_, 10);
    }

private:
    // runtime data
    std::vector<Tree> trees_;
    int next_tree_id_;

    // parameters
    std::string scan_topic_;
    std::string detected_topic_;
    double max_distance_;
    double cluster_dist_threshold_;
    size_t min_cluster_size_;
    double match_threshold_;
    double curvature_ratio_threshold_;
    double curvature_abs_threshold_;
    double fit_max_rmse_;
    double fit_min_radius_;
    double fit_max_radius_;
    bool accept_fallback_;
    bool debug_reasons_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    void load_parameters()
    {
        scan_topic_ = get_parameter("scan_topic").as_string();
        detected_topic_ = get_parameter("detected_topic").as_string();
        max_distance_ = get_parameter("max_distance").as_double();
        cluster_dist_threshold_ = get_parameter("cluster_dist_threshold").as_double();
        min_cluster_size_ = static_cast<size_t>(get_parameter("min_cluster_size").as_int());
        match_threshold_ = get_parameter("match_threshold").as_double();
        curvature_ratio_threshold_ = get_parameter("curvature_ratio_threshold").as_double();
        curvature_abs_threshold_ = get_parameter("curvature_abs_threshold").as_double();
        fit_max_rmse_ = get_parameter("fit_max_rmse").as_double();
        fit_min_radius_ = get_parameter("fit_min_radius").as_double();
        fit_max_radius_ = get_parameter("fit_max_radius").as_double();
        accept_fallback_ = get_parameter("accept_fallback").as_bool();
        debug_reasons_ = get_parameter("debug_reasons").as_bool();
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // reload parameters in case they changed
        load_parameters();

        auto points = preprocess_lidar(msg);
        auto clusters = cluster_points(points);

        if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "points=%zu clusters=%zu", points.size(), clusters.size());

        for (const auto &cluster : clusters) {
            if (cluster.size() < min_cluster_size_) {
                if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "REJECT: too few points (%zu < %zu)", cluster.size(), min_cluster_size_);
                continue;
            }

            // require curvature unless fallback allowed
            float chord, max_perp, ratio;
            compute_chord_perp(cluster, chord, max_perp, ratio);
            if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "cluster size=%zu chord=%.3f max_perp=%.3f ratio=%.4f",
                                             cluster.size(), chord, max_perp, ratio);

            if (!cluster_is_curved(cluster) && !accept_fallback_) {
                if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "REJECT: not curved");
                continue;
            }

            // Fit circle (preferred)
            double cx=0.0, cy=0.0, radius=0.0, fit_rmse=0.0;
            bool fit_ok = fit_circle_kasa(cluster, cx, cy, radius, fit_rmse);

            bool accepted = false;
            double diameter = 0.0;
            if (fit_ok && fit_rmse <= fit_max_rmse_ && radius >= fit_min_radius_ && radius <= fit_max_radius_) {
                diameter = 2.0 * radius;
                accepted = true;
                if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "ACCEPT fit cx=%.3f cy=%.3f r=%.3f rmse=%.4f", cx, cy, radius, fit_rmse);
            } else {
                if (debug_reasons_) {
                    if (fit_ok) RCLCPP_DEBUG(get_logger(), "Fit outside bounds rmse=%.4f r=%.3f", fit_rmse, radius);
                    else RCLCPP_DEBUG(get_logger(), "Fit failed");
                }
                // fallback: centroid + max pairwise distance (perimeter)
                auto cen = compute_centroid(cluster);
                double fallback_diam = estimate_diameter(cluster);
                if (fallback_diam >= 2.0 * fit_min_radius_ && fallback_diam <= 2.0 * fit_max_radius_) {
                    cx = cen.first; cy = cen.second; diameter = fallback_diam; accepted = true;
                    if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "ACCEPT fallback within bounds diam=%.3f", diameter);
                } else if (accept_fallback_) {
                    cx = cen.first; cy = cen.second; diameter = fallback_diam; accepted = true;
                    if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "ACCEPT relaxed fallback diam=%.3f", diameter);
                } else {
                    if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "REJECT fallback diam=%.3f", fallback_diam);
                }
            }

            if (!accepted) continue;

            int match_idx = find_matching_tree(cx, cy);
            if (match_idx >= 0) {
                Tree &t = trees_[match_idx];
                t.count++;
                t.x_avg += (cx - t.x_avg) / t.count;
                t.y_avg += (cy - t.y_avg) / t.count;
                t.diameters.push_back(diameter);
                RCLCPP_INFO(get_logger(), "updated id=%d diam=%.3f obs=%d", t.id, diameter, t.count);
            } else {
                trees_.emplace_back(next_tree_id_++, cx, cy, diameter);
                RCLCPP_INFO(get_logger(), "added id=%d center=(%.3f,%.3f) diam=%.3f", next_tree_id_-1, cx, cy, diameter);
            }
        }

        publish_tree_array();
    }

    std::vector<std::pair<double,double>> preprocess_lidar(const sensor_msgs::msg::LaserScan::SharedPtr& msg) const
    {
        std::vector<std::pair<double,double>> pts;
        double angle = msg->angle_min;
        for (const auto &r : msg->ranges) {
            if (std::isfinite(r) && r > 0.0 && r <= max_distance_) {
                double x = r * std::cos(angle);
                double y = r * std::sin(angle);
                pts.emplace_back(x, y);
            }
            angle += msg->angle_increment;
        }
        return pts;
    }

    std::vector<std::vector<std::pair<double,double>>> cluster_points(const std::vector<std::pair<double,double>>& pts) const
    {
        std::vector<std::vector<std::pair<double,double>>> clusters;
        if (pts.empty()) return clusters;
        std::vector<std::pair<double,double>> cluster;
        for (size_t i = 0; i < pts.size(); ++i) {
            if (cluster.empty()) {
                cluster.push_back(pts[i]);
            } else {
                double dx = pts[i].first - cluster.back().first;
                double dy = pts[i].second - cluster.back().second;
                double d = std::hypot(dx, dy);
                if (d < cluster_dist_threshold_) {
                    cluster.push_back(pts[i]);
                } else {
                    if (cluster.size() >= min_cluster_size_) clusters.push_back(cluster);
                    cluster.clear();
                    cluster.push_back(pts[i]);
                }
            }
        }
        if (cluster.size() >= min_cluster_size_) clusters.push_back(cluster);
        return clusters;
    }

    void compute_chord_perp(const std::vector<std::pair<double,double>>& cluster, float &chord, float &max_perp, float &ratio) const
    {
        chord = max_perp = ratio = 0.0f;
        if (cluster.size() < 2) return;
        const auto &p0 = cluster.front();
        const auto &pN = cluster.back();
        double dx = pN.first - p0.first;
        double dy = pN.second - p0.second;
        double c = std::hypot(dx, dy);
        chord = static_cast<float>(c);
        if (c < 1e-8) return;
        double mp = 0.0;
        for (const auto &p : cluster) {
            double cross = std::abs((p.first - p0.first) * dy - (p.second - p0.second) * dx);
            double perp = cross / c;
            if (perp > mp) mp = perp;
        }
        max_perp = static_cast<float>(mp);
        ratio = static_cast<float>( mp / (c > 0.0 ? c : 1.0) );
    }

    bool cluster_is_curved(const std::vector<std::pair<double,double>>& cluster) const
    {
        if (cluster.size() < 3) return false;
        float chord, max_perp, ratio;
        compute_chord_perp(cluster, chord, max_perp, ratio);
        return (max_perp >= static_cast<float>(curvature_abs_threshold_) && ratio >= static_cast<float>(curvature_ratio_threshold_));
    }

    // Solve 3x3 linear system in-place (augmented matrix a[3][4]) -> x[3]
    static bool solve3x3(double a[3][4], double x[3])
    {
        const int N = 3;
        for (int i = 0; i < N; ++i) {
            int piv = i;
            for (int r = i+1; r < N; ++r)
                if (std::abs(a[r][i]) > std::abs(a[piv][i])) piv = r;
            if (std::abs(a[piv][i]) < 1e-12) return false;
            if (piv != i) for (int c = i; c < 4; ++c) std::swap(a[i][c], a[piv][c]);
            double diag = a[i][i];
            for (int c = i; c < 4; ++c) a[i][c] /= diag;
            for (int r = i+1; r < N; ++r) {
                double fac = a[r][i];
                for (int c = i; c < 4; ++c) a[r][c] -= fac * a[i][c];
            }
        }
        for (int i = N-1; i >= 0; --i) {
            double val = a[i][3];
            for (int j = i+1; j < N; ++j) val -= a[i][j] * x[j];
            x[i] = val;
        }
        return true;
    }

    // Kåsa algebraic circle fit (least-squares)
    bool fit_circle_kasa(const std::vector<std::pair<double,double>>& pts, double &cx, double &cy, double &r, double &rmse) const
    {
        size_t n = pts.size();
        if (n < 3) return false;
        double Sx=0, Sy=0, Sxx=0, Syy=0, Sxy=0, Sr=0, Sxxx=0, Sxxy=0, Sxyy=0, Syyy=0;
        for (const auto &p : pts) {
            double x = p.first;
            double y = p.second;
            double xx = x*x;
            double yy = y*y;
            Sx += x; Sy += y;
            Sxx += xx; Syy += yy; Sxy += x*y;
            Sr += xx + yy;
            Sxxx += x*xx; Sxxy += x*yy; Sxyy += y*xx; Syyy += y*yy;
        }

        double A[3][4];
        A[0][0] = Sxx; A[0][1] = Sxy; A[0][2] = Sx; A[0][3] = Sxxx + Sxyy;
        A[1][0] = Sxy; A[1][1] = Syy; A[1][2] = Sy; A[1][3] = Sxxy + Syyy;
        A[2][0] = Sx;  A[2][1] = Sy;  A[2][2] = static_cast<double>(n); A[2][3] = Sr;

        double sol[3] = {0.0,0.0,0.0};
        if (!solve3x3(A, sol)) return false;
        double D = sol[0], E = sol[1], F = sol[2];

        cx = -D / 2.0;
        cy = -E / 2.0;
        double rr = cx*cx + cy*cy - F;
        if (rr <= 0.0) return false;
        r = std::sqrt(rr);

        double se = 0.0;
        for (const auto &p : pts) {
            double d = std::hypot(p.first - cx, p.second - cy);
            double err = d - r;
            se += err * err;
        }
        rmse = std::sqrt(se / static_cast<double>(n));
        return true;
    }

    std::pair<double,double> compute_centroid(const std::vector<std::pair<double,double>>& cluster) const
    {
        double sx = 0.0, sy = 0.0;
        for (const auto &p : cluster) { sx += p.first; sy += p.second; }
        return { sx / cluster.size(), sy / cluster.size() };
    }

    double estimate_diameter(const std::vector<std::pair<double,double>>& cluster) const
    {
        double maxd = 0.0;
        for (size_t i = 0; i < cluster.size(); ++i) {
            for (size_t j = i+1; j < cluster.size(); ++j) {
                double dx = cluster[i].first - cluster[j].first;
                double dy = cluster[i].second - cluster[j].second;
                double d = std::hypot(dx, dy);
                if (d > maxd) maxd = d;
            }
        }
        return maxd;
    }

    int find_matching_tree(double x, double y) const
    {
        for (size_t i = 0; i < trees_.size(); ++i) {
            double dx = trees_[i].x_avg - x;
            double dy = trees_[i].y_avg - y;
            double dist = std::hypot(dx, dy);
            if (dist < match_threshold_) return static_cast<int>(i);
        }
        return -1;
    }

    void publish_tree_array()
    {
        std_msgs::msg::String msg;
        std::ostringstream oss;
        oss << "Detected trees (" << trees_.size() << "): ";
        for (size_t i = 0; i < trees_.size(); ++i) {
            oss << "[id=" << trees_[i].id
                << ", x=" << trees_[i].x_avg
                << ", y=" << trees_[i].y_avg
                << ", avg_diam=" << trees_[i].average_diameter()
                << ", latest_diam=" << trees_[i].latest_diameter()
                << ", obs=" << trees_[i].count << "]";
            if (i + 1 != trees_.size()) oss << ", ";
        }
        msg.data = oss.str();
        publisher_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarTreeDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
```// filepath: /home/student/41068_ws/src/41068RS12025-Group18/lidar-tree-detector/src/lidar_tree_detector_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <limits>

struct Tree
{
    int id;
    double x_avg;
    double y_avg;
    std::vector<double> diameters;
    int count;

    Tree(int id_, double x, double y, double diameter)
        : id(id_), x_avg(x), y_avg(y), diameters{diameter}, count(1) {}

    double latest_diameter() const { return diameters.empty() ? 0.0 : diameters.back(); }
    double average_diameter() const {
        double sum = 0.0;
        for (double d : diameters) sum += d;
        return diameters.empty() ? 0.0 : sum / diameters.size();
    }
};

class LidarTreeDetectorNode : public rclcpp::Node
{
public:
    LidarTreeDetectorNode()
    : Node("lidar_tree_detector"), next_tree_id_(1)
    {
        // declare tunable parameters
        declare_parameter<std::string>("scan_topic", "/scan");
        declare_parameter<std::string>("detected_topic", "/detected_trees");
        declare_parameter<double>("max_distance", 15.0);
        declare_parameter<double>("cluster_dist_threshold", 0.6);
        declare_parameter<int>("min_cluster_size", 5);
        declare_parameter<double>("match_threshold", 1.0);
        declare_parameter<double>("curvature_ratio_threshold", 0.03);
        declare_parameter<double>("curvature_abs_threshold", 0.03);
        declare_parameter<double>("fit_max_rmse", 0.08);
        declare_parameter<double>("fit_min_radius", 0.02);
        declare_parameter<double>("fit_max_radius", 1.2);
        declare_parameter<bool>("accept_fallback", true);
        declare_parameter<bool>("debug_reasons", false);

        load_parameters();

        RCLCPP_INFO(get_logger(), "Started lidar_tree_detector (scan=%s detected=%s)",
                    scan_topic_.c_str(), detected_topic_.c_str());

        subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10,
            std::bind(&LidarTreeDetectorNode::lidar_callback, this, std::placeholders::_1)
        );
        publisher_ = create_publisher<std_msgs::msg::String>(detected_topic_, 10);
    }

private:
    // runtime data
    std::vector<Tree> trees_;
    int next_tree_id_;

    // parameters
    std::string scan_topic_;
    std::string detected_topic_;
    double max_distance_;
    double cluster_dist_threshold_;
    size_t min_cluster_size_;
    double match_threshold_;
    double curvature_ratio_threshold_;
    double curvature_abs_threshold_;
    double fit_max_rmse_;
    double fit_min_radius_;
    double fit_max_radius_;
    bool accept_fallback_;
    bool debug_reasons_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    void load_parameters()
    {
        scan_topic_ = get_parameter("scan_topic").as_string();
        detected_topic_ = get_parameter("detected_topic").as_string();
        max_distance_ = get_parameter("max_distance").as_double();
        cluster_dist_threshold_ = get_parameter("cluster_dist_threshold").as_double();
        min_cluster_size_ = static_cast<size_t>(get_parameter("min_cluster_size").as_int());
        match_threshold_ = get_parameter("match_threshold").as_double();
        curvature_ratio_threshold_ = get_parameter("curvature_ratio_threshold").as_double();
        curvature_abs_threshold_ = get_parameter("curvature_abs_threshold").as_double();
        fit_max_rmse_ = get_parameter("fit_max_rmse").as_double();
        fit_min_radius_ = get_parameter("fit_min_radius").as_double();
        fit_max_radius_ = get_parameter("fit_max_radius").as_double();
        accept_fallback_ = get_parameter("accept_fallback").as_bool();
        debug_reasons_ = get_parameter("debug_reasons").as_bool();
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // reload parameters in case they changed
        load_parameters();

        auto points = preprocess_lidar(msg);
        auto clusters = cluster_points(points);

        if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "points=%zu clusters=%zu", points.size(), clusters.size());

        for (const auto &cluster : clusters) {
            if (cluster.size() < min_cluster_size_) {
                if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "REJECT: too few points (%zu < %zu)", cluster.size(), min_cluster_size_);
                continue;
            }

            // require curvature unless fallback allowed
            float chord, max_perp, ratio;
            compute_chord_perp(cluster, chord, max_perp, ratio);
            if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "cluster size=%zu chord=%.3f max_perp=%.3f ratio=%.4f",
                                             cluster.size(), chord, max_perp, ratio);

            if (!cluster_is_curved(cluster) && !accept_fallback_) {
                if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "REJECT: not curved");
                continue;
            }

            // Fit circle (preferred)
            double cx=0.0, cy=0.0, radius=0.0, fit_rmse=0.0;
            bool fit_ok = fit_circle_kasa(cluster, cx, cy, radius, fit_rmse);

            bool accepted = false;
            double diameter = 0.0;
            if (fit_ok && fit_rmse <= fit_max_rmse_ && radius >= fit_min_radius_ && radius <= fit_max_radius_) {
                diameter = 2.0 * radius;
                accepted = true;
                if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "ACCEPT fit cx=%.3f cy=%.3f r=%.3f rmse=%.4f", cx, cy, radius, fit_rmse);
            } else {
                if (debug_reasons_) {
                    if (fit_ok) RCLCPP_DEBUG(get_logger(), "Fit outside bounds rmse=%.4f r=%.3f", fit_rmse, radius);
                    else RCLCPP_DEBUG(get_logger(), "Fit failed");
                }
                // fallback: centroid + max pairwise distance (perimeter)
                auto cen = compute_centroid(cluster);
                double fallback_diam = estimate_diameter(cluster);
                if (fallback_diam >= 2.0 * fit_min_radius_ && fallback_diam <= 2.0 * fit_max_radius_) {
                    cx = cen.first; cy = cen.second; diameter = fallback_diam; accepted = true;
                    if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "ACCEPT fallback within bounds diam=%.3f", diameter);
                } else if (accept_fallback_) {
                    cx = cen.first; cy = cen.second; diameter = fallback_diam; accepted = true;
                    if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "ACCEPT relaxed fallback diam=%.3f", diameter);
                } else {
                    if (debug_reasons_) RCLCPP_DEBUG(get_logger(), "REJECT fallback diam=%.3f", fallback_diam);
                }
            }

            if (!accepted) continue;

            int match_idx = find_matching_tree(cx, cy);
            if (match_idx >= 0) {
                Tree &t = trees_[match_idx];
                t.count++;
                t.x_avg += (cx - t.x_avg) / t.count;
                t.y_avg += (cy - t.y_avg) / t.count;
                t.diameters.push_back(diameter);
                RCLCPP_INFO(get_logger(), "updated id=%d diam=%.3f obs=%d", t.id, diameter, t.count);
            } else {
                trees_.emplace_back(next_tree_id_++, cx, cy, diameter);
                RCLCPP_INFO(get_logger(), "added id=%d center=(%.3f,%.3f) diam=%.3f", next_tree_id_-1, cx, cy, diameter);
            }
        }

        publish_tree_array();
    }

    std::vector<std::pair<double,double>> preprocess_lidar(const sensor_msgs::msg::LaserScan::SharedPtr& msg) const
    {
        std::vector<std::pair<double,double>> pts;
        double angle = msg->angle_min;
        for (const auto &r : msg->ranges) {
            if (std::isfinite(r) && r > 0.0 && r <= max_distance_) {
                double x = r * std::cos(angle);
                double y = r * std::sin(angle);
                pts.emplace_back(x, y);
            }
            angle += msg->angle_increment;
        }
        return pts;
    }

    std::vector<std::vector<std::pair<double,double>>> cluster_points(const std::vector<std::pair<double,double>>& pts) const
    {
        std::vector<std::vector<std::pair<double,double>>> clusters;
        if (pts.empty()) return clusters;
        std::vector<std::pair<double,double>> cluster;
        for (size_t i = 0; i < pts.size(); ++i) {
            if (cluster.empty()) {
                cluster.push_back(pts[i]);
            } else {
                double dx = pts[i].first - cluster.back().first;
                double dy = pts[i].second - cluster.back().second;
                double d = std::hypot(dx, dy);
                if (d < cluster_dist_threshold_) {
                    cluster.push_back(pts[i]);
                } else {
                    if (cluster.size() >= min_cluster_size_) clusters.push_back(cluster);
                    cluster.clear();
                    cluster.push_back(pts[i]);
                }
            }
        }
        if (cluster.size() >= min_cluster_size_) clusters.push_back(cluster);
        return clusters;
    }

    void compute_chord_perp(const std::vector<std::pair<double,double>>& cluster, float &chord, float &max_perp, float &ratio) const
    {
        chord = max_perp = ratio = 0.0f;
        if (cluster.size() < 2) return;
        const auto &p0 = cluster.front();
        const auto &pN = cluster.back();
        double dx = pN.first - p0.first;
        double dy = pN.second - p0.second;
        double c = std::hypot(dx, dy);
        chord = static_cast<float>(c);
        if (c < 1e-8) return;
        double mp = 0.0;
        for (const auto &p : cluster) {
            double cross = std::abs((p.first - p0.first) * dy - (p.second - p0.second) * dx);
            double perp = cross / c;
            if (perp > mp) mp = perp;
        }
        max_perp = static_cast<float>(mp);
        ratio = static_cast<float>( mp / (c > 0.0 ? c : 1.0) );
    }

    bool cluster_is_curved(const std::vector<std::pair<double,double>>& cluster) const
    {
        if (cluster.size() < 3) return false;
        float chord, max_perp, ratio;
        compute_chord_perp(cluster, chord, max_perp, ratio);
        return (max_perp >= static_cast<float>(curvature_abs_threshold_) && ratio >= static_cast<float>(curvature_ratio_threshold_));
    }

    // Solve 3x3 linear system in-place (augmented matrix a[3][4]) -> x[3]
    static bool solve3x3(double a[3][4], double x[3])
    {
        const int N = 3;
        for (int i = 0; i < N; ++i) {
            int piv = i;
            for (int r = i+1; r < N; ++r)
                if (std::abs(a[r][i]) > std::abs(a[piv][i])) piv = r;
            if (std::abs(a[piv][i]) < 1e-12) return false;
            if (piv != i) for (int c = i; c < 4; ++c) std::swap(a[i][c], a[piv][c]);
            double diag = a[i][i];
            for (int c = i; c < 4; ++c) a[i][c] /= diag;
            for (int r = i+1; r < N; ++r) {
                double fac = a[r][i];
                for (int c = i; c < 4; ++c) a[r][c] -= fac * a[i][c];
            }
        }
        for (int i = N-1; i >= 0; --i) {
            double val = a[i][3];
            for (int j = i+1; j < N; ++j) val -= a[i][j] * x[j];
            x[i] = val;
        }
        return true;
    }

    // Kåsa algebraic circle fit (least-squares)
    bool fit_circle_kasa(const std::vector<std::pair<double,double>>& pts, double &cx, double &cy, double &r, double &rmse) const
    {
        size_t n = pts.size();
        if (n < 3) return false;
        double Sx=0, Sy=0, Sxx=0, Syy=0, Sxy=0, Sr=0, Sxxx=0, Sxxy=0, Sxyy=0, Syyy=0;
        for (const auto &p : pts) {
            double x = p.first;
            double y = p.second;
            double xx = x*x;
            double yy = y*y;
            Sx += x; Sy += y;
            Sxx += xx; Syy += yy; Sxy += x*y;
            Sr += xx + yy;
            Sxxx += x*xx; Sxxy += x*yy; Sxyy += y*xx; Syyy += y*yy;
        }

        double A[3][4];
        A[0][0] = Sxx; A[0][1] = Sxy; A[0][2] = Sx; A[0][3] = Sxxx + Sxyy;
        A[1][0] = Sxy; A[1][1] = Syy; A[1][2] = Sy; A[1][3] = Sxxy + Syyy;
        A[2][0] = Sx;  A[2][1] = Sy;  A[2][2] = static_cast<double>(n); A[2][3] = Sr;

        double sol[3] = {0.0,0.0,0.0};
        if (!solve3x3(A, sol)) return false;
        double D = sol[0], E = sol[1], F = sol[2];

        cx = -D / 2.0;
        cy = -E / 2.0;
        double rr = cx*cx + cy*cy - F;
        if (rr <= 0.0) return false;
        r = std::sqrt(rr);

        double se = 0.0;
        for (const auto &p : pts) {
            double d = std::hypot(p.first - cx, p.second - cy);
            double err = d - r;
            se += err * err;
        }
        rmse = std::sqrt(se / static_cast<double>(n));
        return true;
    }

    std::pair<double,double> compute_centroid(const std::vector<std::pair<double,double>>& cluster) const
    {
        double sx = 0.0, sy = 0.0;
        for (const auto &p : cluster) { sx += p.first; sy += p.second; }
        return { sx / cluster.size(), sy / cluster.size() };
    }

    double estimate_diameter(const std::vector<std::pair<double,double>>& cluster) const
    {
        double maxd = 0.0;
        for (size_t i = 0; i < cluster.size(); ++i) {
            for (size_t j = i+1; j < cluster.size(); ++j) {
                double dx = cluster[i].first - cluster[j].first;
                double dy = cluster[i].second - cluster[j].second;
                double d = std::hypot(dx, dy);
                if (d > maxd) maxd = d;
            }
        }
        return maxd;
    }

    int find_matching_tree(double x, double y) const
    {
        for (size_t i = 0; i < trees_.size(); ++i) {
            double dx = trees_[i].x_avg - x;
            double dy = trees_[i].y_avg - y;
            double dist = std::hypot(dx, dy);
            if (dist < match_threshold_) return static_cast<int>(i);
        }
        return -1;
    }

    void publish_tree_array()
    {
        std_msgs::msg::String msg;
        std::ostringstream oss;
        oss << "Detected trees (" << trees_.size() << "): ";
        for (size_t i = 0; i < trees_.size(); ++i) {
            oss << "[id=" << trees_[i].id
                << ", x=" << trees_[i].x_avg
                << ", y=" << trees_[i].y_avg
                << ", avg_diam=" << trees_[i].average_diameter()
                << ", latest_diam=" << trees_[i].latest_diameter()
                << ", obs=" << trees_[i].count << "]";
            if (i + 1 != trees_.size()) oss << ", ";
        }
        msg.data = oss.str();
        publisher_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarTreeDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
#pragma once
#include <vector>
#include <geometry_msgs/msg/point.hpp>

class TrunkDetector {
public:
    struct Result {
        double width_m;              // Estimated trunk width in meters
        geometry_msgs::msg::Point center; // Trunk center position
        bool valid;
    };

    // Process raw lidar ranges
    Result detectTrunk(const std::vector<float>& ranges, float angle_min, float angle_increment);

private:
    std::pair<int, int> findCluster(const std::vector<float>& ranges, float distance_threshold);
    double calculateWidth(const std::vector<float>& ranges, int start_idx, int end_idx, float angle_min, float angle_increment);
};

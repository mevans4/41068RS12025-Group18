#pragma once
#include <vector>
#include <geometry_msgs/msg/point.hpp>

class TrunkDetector {
public:
    struct Result {
        double diameter_m;                      // Estimated trunk diameter in meters
        geometry_msgs::msg::Point center;       // Approximate trunk center
        bool valid;
    };

    // Main function to process LiDAR scan ranges
    Result detectTrunk(const std::vector<float>& ranges, float angle_min, float angle_increment);

private:
    std::pair<int, int> findCluster(const std::vector<float>& ranges, float distance_threshold);
    double calculateDiameter(const std::vector<float>& ranges, int start_idx, int end_idx, float angle_min, float angle_increment);
};

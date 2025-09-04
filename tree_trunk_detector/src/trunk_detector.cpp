#include "tree_trunk_detector/trunk_detector.hpp"
#include <cmath>
#include <limits>

std::pair<int, int> TrunkDetector::findCluster(const std::vector<float>& ranges, float distance_threshold) {
    int start = -1, end = -1;
    for (size_t i = 1; i < ranges.size(); ++i) {
        if (std::abs(ranges[i] - ranges[i - 1]) > distance_threshold) {
            if (start == -1) start = i;
            end = i;
        }
    }
    return {start, end};
}

double TrunkDetector::calculateWidth(const std::vector<float>& ranges, int start_idx, int end_idx,
                                     float angle_min, float angle_increment) {
    if (start_idx < 0 || end_idx < 0 || start_idx >= end_idx) return 0.0;

    float r1 = ranges[start_idx];
    float r2 = ranges[end_idx];
    float angle_diff = (end_idx - start_idx) * angle_increment;

    // Law of Cosines to compute width
    return std::sqrt(r1 * r1 + r2 * r2 - 2 * r1 * r2 * std::cos(angle_diff));
}

TrunkDetector::Result TrunkDetector::detectTrunk(const std::vector<float>& ranges, float angle_min, float angle_increment) {
    Result result;
    result.valid = false;

    auto [start, end] = findCluster(ranges, 0.1); // 10 cm cluster break
    if (start == -1 || end == -1) return result;

    result.width_m = calculateWidth(ranges, start, end, angle_min, angle_increment);
    result.valid = result.width_m > 0.0;
    return result;
}

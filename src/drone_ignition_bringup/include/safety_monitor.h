
#ifndef DRONE_SAFETY_MONITOR_H
#define DRONE_SAFETY_MONITOR_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace drone
{

class SafetyMonitor
{
public:
  SafetyMonitor();
  void updateLimits(double max_altitude, double max_velocity, double max_distance);
  bool checkSafetyLimits(const geometry_msgs::msg::PoseStamped& pose,
                        const geometry_msgs::msg::Twist& velocity);
  void setHomePosition(const geometry_msgs::msg::PoseStamped& home);
  bool isEmergencyRequired() const;
  std::string getLastViolation() const;

private:
  geometry_msgs::msg::PoseStamped home_position_;
  double max_altitude_;
  double max_velocity_;
  double max_distance_from_home_;
  bool emergency_required_;
  std::string last_violation_;
};

}  // namespace drone

#endif  // DRONE_SAFETY_MONITOR_H

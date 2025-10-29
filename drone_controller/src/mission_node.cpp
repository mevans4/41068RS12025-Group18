#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class MissionNode : public rclcpp::Node
{
public:
    MissionNode() : Node("mission_node")
    {
    client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this,
      "navigate_through_poses");

    while (!client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for action server to be available...");
    }
    RCLCPP_INFO(this->get_logger(), "Action server available.");

    send_mission();
    }

private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_;
    
    void send_mission()
    {
        NavigateThroughPoses::Goal goal;

        geometry_msgs::msg::PoseStamped pose1, pose2, pose3;

        // pose1.header.frame_id = "map";
        // pose1.pose.position.x = -2.0;
        // pose1.pose.position.y = 12.0;
        // pose1.pose.orientation.z = 0.707;
        // pose1.pose.orientation.w = 0.707;

        // pose2.header.frame_id = "map";
        // pose2.pose.position.x = 2.0;
        // pose2.pose.position.y = 12.0;
        // pose2.pose.orientation.z = -0.707;
        // pose2.pose.orientation.w = 0.707;

        // pose3.header.frame_id = "map";
        // pose3.pose.position.x = 2.0;
        // pose3.pose.position.y = -12.0;
        // pose3.pose.orientation.z = -0.707;
        // pose3.pose.orientation.w = 0.707;

        pose1.header.frame_id = "map";
        pose1.pose.position.x = 0.5;
        pose1.pose.position.y = 7.2;
        pose1.pose.orientation.z = 0.2;
        pose1.pose.orientation.w = 1.0;

        pose2.header.frame_id = "map";
        pose2.pose.position.x = 2.0;
        pose2.pose.position.y = 3.5;
        pose2.pose.orientation.z = -1.0;
        pose2.pose.orientation.w = 0.2;

        pose3.header.frame_id = "map";
        pose3.pose.position.x = -20.0;
        pose3.pose.position.y = -4.5;
        pose3.pose.orientation.z = -1.0;
        pose3.pose.orientation.w = 0.2;

        // Fill in the goal details
        goal.poses.push_back(pose1);
        goal.poses.push_back(pose2);
        goal.poses.push_back(pose3);

        auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [this](GoalHandle::SharedPtr,
               const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Received feedback");
            };

        // Send the goal to the action server
        client_->async_send_goal(goal);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
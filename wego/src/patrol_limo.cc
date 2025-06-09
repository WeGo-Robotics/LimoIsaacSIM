#include <functional>
#include <future>
#include <memory>
#include <chrono>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace wego
{
class PatrolLimo : public rclcpp::Node
{
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<NavigateToPose>;
        
        explicit PatrolLimo(const rclcpp::NodeOptions & options)
        : Node("patrol_limo", options)
        {
            this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
                this,
                "navigate_to_pose"
            );

            while(!client_ptr_->wait_for_action_server(5s)){
                RCLCPP_INFO(get_logger(), "Waiting for 'navigate_to_pose' action server...");
            }

            loadWaypoints();
            sendNextGoal();
        }

    private:
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
        std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
        size_t current_goal_index_=0;
        
        void loadWaypoints()
        {
            geometry_msgs::msg::PoseStamped pose1;
            pose1.header.frame_id = "map";
            pose1.pose.position.x = 0.4317;
            pose1.pose.orientation.w = 1.0;

            geometry_msgs::msg::PoseStamped pose2;
            pose2.header.frame_id = "map";
            pose2.pose.position.x = 0.8016;
            pose2.pose.position.y = 0.2698;
            pose2.pose.orientation.w = 1.0;

            geometry_msgs::msg::PoseStamped pose3;
            pose3.header.frame_id = "map";
            pose3.pose.position.x = 1.496118;
            pose3.pose.position.y = 0.238669;
            pose3.pose.orientation.w = 0.69692;
            pose3.pose.orientation.z = -0.71714;

            geometry_msgs::msg::PoseStamped pose4;
            pose4.header.frame_id = "map";
            pose4.pose.position.x = 1.40994;
            pose4.pose.position.y = -0.6186;
            pose4.pose.orientation.w = 0.0;
            pose4.pose.orientation.z = -1.0;

            geometry_msgs::msg::PoseStamped pose5;
            pose5.header.frame_id = "map";
            pose5.pose.position.x = -0.86808;
            pose5.pose.position.y = -0.62198;
            pose5.pose.orientation.w = -0.709239;
            pose5.pose.orientation.z = -0.70496;

            geometry_msgs::msg::PoseStamped pose6;
            pose6.header.frame_id = "map";
            pose6.pose.position.x = -0.8079;
            pose6.pose.position.y = 1.67594;
            pose6.pose.orientation.w = 1.0;
            pose6.pose.orientation.z = 0.0;

            geometry_msgs::msg::PoseStamped pose7;
            pose7.header.frame_id = "map";
            pose7.pose.position.x = 0.48036;
            pose7.pose.position.y = 1.65949;
            pose7.pose.orientation.w = -0.68308;
            pose7.pose.orientation.z = 0.73033;

            waypoints_.push_back(pose1);
            waypoints_.push_back(pose2);
            waypoints_.push_back(pose3);
            waypoints_.push_back(pose4);
            waypoints_.push_back(pose5);
            waypoints_.push_back(pose6);
            waypoints_.push_back(pose7);
        }

        void sendNextGoal()
        {
            if (waypoints_.empty()) {
                RCLCPP_WARN(get_logger(), "No waypoints defined.");
                return;
            }

            auto & next_pose = waypoints_[current_goal_index_];
            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose = next_pose;
            goal_msg.pose.header.stamp = now();

            RCLCPP_INFO(get_logger(), "Sending goal %zu: (%.2f, %.2f)", current_goal_index_,
                next_pose.pose.position.x, next_pose.pose.position.y);
            
            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

            send_goal_options.result_callback =
                std::bind(&PatrolLimo::resultCallback, this, std::placeholders::_1);
            
            client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

        void resultCallback(const GoalHandleNavigation::WrappedResult & result)
        {
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(get_logger(), "Goal %zu succeeded!", current_goal_index_);
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(get_logger(), "Goal %zu aborted!", current_goal_index_);
              rclcpp::shutdown();
              return;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_WARN(get_logger(), "Goal %zu canceled!", current_goal_index_);
              rclcpp::shutdown();
              return;
            default:
              RCLCPP_ERROR(get_logger(), "Unknown result code");
              rclcpp::shutdown();
              return;
          }
        
          current_goal_index_ = (current_goal_index_ + 1) % waypoints_.size();
          sendNextGoal();
        }
};
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(wego::PatrolLimo)
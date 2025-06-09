#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace wego
{
class IsaacsimJoystickBridge : public rclcpp::Node
{
public:
    IsaacsimJoystickBridge(const rclcpp::NodeOptions & options)
    : Node("isaacsim_joystick_bridge", options)
    {
        this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            10,
            [this](const sensor_msgs::msg::Joy _msg){
                if(_msg.buttons[4]){
                    RCLCPP_INFO(this->get_logger(),"Joystick enabled!");
                    this->maual_convert_flag_=true;
                }
                else{
                    this->maual_convert_flag_=false;
                }
                this->joy_cmd_.linear.x = _msg.axes[1]*0.5;
                this->joy_cmd_.angular.z = _msg.axes[3]*1.5;
            }
        );

        this->auto_cmd_sub_=this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            [this](const geometry_msgs::msg::Twist _msg){this->auto_cmd_ = _msg;}
        );

        this->sim_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/sim/cmd_vel",10);
        auto timer_callback = [this](){
            auto cmd = geometry_msgs::msg::Twist();
            if(this->maual_convert_flag_){cmd = joy_cmd_;}
            else{cmd = auto_cmd_;}
            this->sim_cmd_pub_->publish(cmd);
        };

        timer_ = this->create_wall_timer(
            100ms,
            timer_callback
        );
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr sim_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool maual_convert_flag_=false;
    geometry_msgs::msg::Twist joy_cmd_;
    geometry_msgs::msg::Twist auto_cmd_;
};
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(wego::IsaacsimJoystickBridge)
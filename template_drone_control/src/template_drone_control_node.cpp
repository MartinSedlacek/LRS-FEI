#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

using namespace std::chrono_literals;

class TemplateDroneControl : public rclcpp::Node
{
public:
    TemplateDroneControl() : Node("template_drone_control_node")
    {
        // Set up ROS publishers, subscribers and service clients
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&TemplateDroneControl::state_cb, this, std::placeholders::_1));
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        // Wait for MAVROS SITL connection
        while (rclcpp::ok() && !current_state_.connected)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }

        mavros_msgs::srv::SetMode::Request guided_set_mode_req;
        guided_set_mode_req.custom_mode = "GUIDED";
        while (!set_mode_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        }
        auto result = set_mode_client_->async_send_request(std::make_shared<mavros_msgs::srv::SetMode::Request>(guided_set_mode_req));

        // TODO: Test if drone state really changed to GUIDED

        // TODO: Arm and Take Off
        RCLCPP_INFO(this->get_logger(), "Sending position command");

        // TODO: Implement position controller and mission commands here
    }

private:
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    mavros_msgs::msg::State current_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateDroneControl>());
    rclcpp::shutdown();
    return 0;
}

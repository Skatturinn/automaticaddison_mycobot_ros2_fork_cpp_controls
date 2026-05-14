// Author: el3403
// Function: This node implements a closed-loop position controller for a 6-DOF robotic arm.
// Input: Joint states from /joint_states and target setpoints from /controller/joint_setpoints.
// Output: Calculated position commands published to /forward_position_controller/commands.

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

// Eigen for Matrix Operations (Pseudo-inverse, cross-coupling)
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nlc_cpp_lib/controllers.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------
// ROS 2 Node
// ---------------------------------------------------------
class in_out_demo : public rclcpp::Node {
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    
    /*
     * Author: el3403
     * Pseudo-code:
     * 1. Initialize publishers for hardware commands and subscribers for joint states/setpoints.
     * 2. Define the 6-DOF joint names and safety velocity bounds.
     * 3. Initialize the 'secant' controller with zeroed states and bounds.
     * 4. Start a wall timer to execute the control loop at 10Hz.
     */
    in_out_demo() : Node("arm_gripper_loop_controller") {
        forward_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);
        
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&in_out_demo::jointStateCallback, this, std::placeholders::_1));
            
        setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/controller/joint_setpoints", 10,
            std::bind(&in_out_demo::setpointCallback, this, std::placeholders::_1));
            
        joint_names_ = {
            "link1_to_link2", 
            "link2_to_link3", 
            "link3_to_link4", 
            "link4_to_link5", 
            "link5_to_link6", 
            "link6_to_link6_flange"
        };

        target_pos_ = std::vector<double>(6, 0.0);

        // Boundaries: These act as VELOCITY caps (rad/s) for safety
        std::vector<controller::min_max> velocity_bounds = {
            {-1.5, 1.5}, {-1.5, 1.5}, {-1.5, 1.5},
            {-1.5, 1.5}, {-1.5, 1.5}, {-1.5, 1.5}
        };
        
        std::vector<double> dummy_meas(6, 0.0);
        my_controller.init_system(dummy_meas, target_pos_, velocity_bounds);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&in_out_demo::controlLoopCallback, this));
    }

private:
    // Author: el3403
    // Function: Parses the incoming JointState message and maps positions to joint names.
    // Input: sensor_msgs::msg::JointState
    // Output: Updates current_joint_positions_ map.
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            current_joint_positions_[msg->name[i]] = msg->position[i];
        }
        got_first_state_ = true;
    }
    
    // Author: el3403
    // Function: Updates the controller's internal target setpoints.
    // Input: std_msgs::msg::Float64MultiArray (6 doubles)
    // Output: Updated controller setpoints.
    void setpointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() == 6) {
            target_pos_ = msg->data;
            my_controller.update_setpoints(target_pos_);
        }
    }

    void sendForwardCommand(const std::vector<double>& commands) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = commands;
        forward_pub_->publish(msg);
    }

    /*
     * Author: el3403
     * Pseudo-code:
     * 1. Validate that joint state data has been received.
     * 2. Extract current joint positions in the specific order required by the controller.
     * 3. Pass measurements to the secant controller and compute the optimal joint velocities.
     * 4. Perform numerical integration: New Position = Current Position + (Velocity * TimeStep).
     * 5. Clamp the resulting position commands to +/- 3.14 radians for safety.
     * 6. Publish the final position vector to the hardware bridge.
     */
    void controlLoopCallback() {
        if (!got_first_state_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for joint states...");
            return;
        }

        std::vector<double> current_meas;
        for (const auto& name : joint_names_) {
            current_meas.push_back(current_joint_positions_[name]);
        }

        my_controller.update_system(current_meas);
        my_controller.compute(); 
        
        // Commands are optimal joint velocities calculated via the Pseudo-Inverse
        std::vector<double> commands = my_controller.getOutputs();

        double Ts = 0.1; // 100ms loop period

        if (current_cmd_pos_.empty()) {
            current_cmd_pos_ = current_meas;
        }

        // Hardware Bridge: Integrate the velocity into the physical position command
        for (size_t i = 0; i < commands.size(); ++i) {
            current_cmd_pos_[i] = current_cmd_pos_[i] + (commands[i] * Ts);
            
            // Hard Joint Position Limits (rads)
            current_cmd_pos_[i] = std::max(-3.14, std::min(current_cmd_pos_[i], 3.14));
        }

        sendForwardCommand(current_cmd_pos_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            "Commanding Joint 1: %f (Target: %f, Error: %f)", 
            current_cmd_pos_.at(0), target_pos_.at(0), target_pos_.at(0) - current_meas.at(0));
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr setpoint_sub_;

    std::vector<std::string> joint_names_;
    std::map<std::string, double> current_joint_positions_;
    bool got_first_state_ = false;

    std::vector<double> target_pos_;
    secant my_controller; 
    std::vector<double> current_cmd_pos_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<in_out_demo>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

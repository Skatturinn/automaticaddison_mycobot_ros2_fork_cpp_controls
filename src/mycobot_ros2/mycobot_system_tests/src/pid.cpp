#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

// Eigen for Matrix Operations
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
    
    in_out_demo() : Node("arm_gripper_loop_controller") {
        forward_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);
        
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&in_out_demo::jointStateCallback, this, std::placeholders::_1));
        
        joint_names_ = {
            "link1_to_link2", 
            "link2_to_link3", 
            "link3_to_link4", 
            "link4_to_link5", 
            "link5_to_link6", 
            "link6_to_link6_flange"
        };

        // Static Target
        //target_pos_ = {1, 1, 1, 1, 0, 0};
        // Listen to the IK Node
        // Listen to the IK Node
        setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/controller/joint_setpoints", 10,
            std::bind(&in_out_demo::setpointCallback, this, std::placeholders::_1));

        // Boundaries: These act as VELOCITY caps (rad/s) for safety
        std::vector<controller::min_max> velocity_bounds = {
            {-1.5, 1.5},
            {-1.5, 1.5},
            {-1.5, 1.5},
            {-1.5, 1.5},
            {-1.5, 1.5},
            {-1.5, 1.5}
        };
        
        target_pos_ = std::vector<double>(6, 0.0);
        
        std::vector<double> dummy_meas(6, 0.0);
        my_controller.init_system(dummy_meas, target_pos_, velocity_bounds);

        // Control loop runs at 10Hz (Ts = 0.1s)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&in_out_demo::controlLoopCallback, this));
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            current_joint_positions_[msg->name[i]] = msg->position[i];
        }
        got_first_state_ = true;
    }
    
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

    void controlLoopCallback() {
        if (!got_first_state_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for joint states...");
            return;
        }
        // NEW: Wait for the IK Streamer to send a target
	    if (target_pos_.empty()) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for IK setpoint...");
		return;
	    }

        std::vector<double> current_meas;
        for (const auto& name : joint_names_) {
            current_meas.push_back(current_joint_positions_[name]);
        }

        my_controller.update_system(current_meas);
        my_controller.compute(); 
        
        // Commands are optimal joint velocities calculated via PID
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
    
    // Instantiating the new PID Controller with Kp=1.0, Ki=0.01, Kd=0.05, and Ts=0.1s
    //pid_controller my_controller{1.0, 0, 0.05, 0.1}; 
    // K=0.5, Ti=0.0, Td=0.05, N=10.0, b=1.0, Tt=0.5, Ts=0.1
    pid_controller my_controller{0.5, 0.0, 0.075, 10.0, 1.0, 0.5, 0.1};
    
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

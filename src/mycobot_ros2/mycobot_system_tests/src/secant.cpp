#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class controller {
public: // Made public so we can use it easily
    struct min_max {
        double min;
        double max;
    };

protected:
    std::vector<double> measurements = {};
    std::vector<double> setpoints = {};
    std::vector<double> output = {};
    std::vector<min_max> boundaries;
    bool initialized = false;

public:
    virtual ~controller() = default; 
    virtual void compute() = 0; 
    
    bool init_system(
        const std::vector<double>& new_measurements,
        const std::vector<double>& new_setpoints,
        const std::vector<min_max>& new_boundaries
    ) {
        if (new_boundaries.size() != new_setpoints.size()) return false;
        
        measurements = new_measurements;
        setpoints = new_setpoints;
        boundaries = new_boundaries;
        output.resize(setpoints.size(), 0.0);
        initialized = true;
        return true;
    }

    bool update_system(const std::vector<double>& new_measurements) {
        if (!initialized || new_measurements.size() != setpoints.size()) return false; 
        measurements = new_measurements;
        return true;    
    }

    void update_setpoints(const std::vector<double>& new_setpoints) {
        setpoints = new_setpoints;
    }

    double clamp(double val, min_max b) {
        return std::max(b.min, std::min(val, b.max));
    }

    std::vector<double> getOutputs() const { return output; }
    bool is_initialized() const { return initialized; }
};

// Secant Class
class secant : public controller {
private:
    double eta;
    double delta;
    std::vector<double> S_hat;
    std::vector<double> u_prev;
    std::vector<double> y_prev;

public:
    secant(double learning_rate = 0.001, double regularization = 0.001) 
        : eta(learning_rate), delta(regularization) {}

    void compute() override {
        if (!initialized) return;

        // Initialize state vectors on first run
        if (S_hat.empty()) {
            S_hat.resize(setpoints.size(), 5.0); // Initial sensitivity guess
            u_prev.resize(setpoints.size(), 0.0);
            y_prev = measurements;
        }

        for (size_t i = 0; i < setpoints.size(); ++i) {
            // Calculate Deltas
            double delta_y = measurements[i] - y_prev[i];
            double delta_u = u_prev[i]; 

            // Secant Update (Equation 1)
            double numerator = delta_y - (S_hat[i] * delta_u);
            double denominator = (delta_u * delta_u) + delta;
            S_hat[i] = S_hat[i] + eta * (numerator / denominator) * delta_u;

            // Calculate Control Update (Equation 2)
            double error = setpoints[i] - measurements[i];
            double u_curr = u_prev[i] + (error / S_hat[i]);
		u_curr *= 0.1;
		u_curr *= measurements[i];
		
            // Clamp output based on boundaries
            output[i] = clamp(u_curr, boundaries[i]);

            // Update history
            y_prev[i] = measurements[i];
            u_prev[i] = u_curr;
            
        }
    }
};

// ROS 2 Node
class in_out_demo : public rclcpp::Node {
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    
    in_out_demo() : Node("arm_gripper_loop_controller") {
        forward_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);
        
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&in_out_demo::jointStateCallback, this, std::placeholders::_1));
        
        gripper_client_ = rclcpp_action::create_client<GripperCommand>(
            this, "/gripper_action_controller/gripper_cmd");

        // Define joint names to ensure consistent ordering
        joint_names_ = {
            "link1_to_link2", 
            "link2_to_link3", 
            "link3_to_link4", 
            "link4_to_link5", 
            "link5_to_link6", 
            "link6_to_link6_flange"
        };

        // Setpoints and Boundaries
        target_pos_ = {1.345, -1.23, 0.264, -0.296, 0.389, -1.5};
        
        // Example bounds (e.g., max velocities or positions depending on your bridge)
        std::vector<controller::min_max> bounds = ({}); 
	std::vector<controller::min_max> bounds = {
	        {-2.87979,2.87979},
	        {-2.35619,2.35619},
	        {-2.61799,2.61799},
	        {-2.53072,2.53072},
	        {-2.87979,2.87979},
	        {-3.31612,3.31612}
	};
        
        // We will initialize the controller with dummy measurements of 0.0 until we get real ones
        std::vector<double> dummy_meas(6, 0.0);
        my_controller.init_system(dummy_meas, target_pos_, bounds);

        // Timer to trigger the control loop at 10Hz (Ts = 0.1s)
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

		// Extract measurements in the correct order
		std::vector<double> current_meas;
		for (const auto& name : joint_names_) {
		    current_meas.push_back(current_joint_positions_[name]);
		}

		// Update System
		my_controller.update_system(current_meas);

		// Compute Secant Output (Velocity)
		my_controller.compute(); 
		std::vector<double> commands = my_controller.getOutputs();

		// FIXED VELOCITY BRIDGE WITH POSITION CLAMP
		double Ts = 0.1; // 100ms loop

		// If this is the first tick, start our command at the current physical position
		if (current_cmd_pos_.empty()) {
		    current_cmd_pos_ = current_meas;
		}

		// Integrate the velocity into the running position command
		for (size_t i = 0; i < commands.size(); ++i) {
		    current_cmd_pos_[i] = current_cmd_pos_[i] + (commands[i] * Ts);
            
            // --> NEW: HARDWARE POSITION CLAMP <--
            // Prevents the position command from exceeding physical joint limits (-3.14 to 3.14 rad)
            current_cmd_pos_[i] = std::max(-3.14, std::min(current_cmd_pos_[i], 3.14));
		}

		sendForwardCommand(current_cmd_pos_);

		// debug print
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
		    "Commanding Joint 1: %f (Target: %f)", current_cmd_pos_.at(0), target_pos_.at(0));
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
    rclcpp::TimerBase::SharedPtr timer_;

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

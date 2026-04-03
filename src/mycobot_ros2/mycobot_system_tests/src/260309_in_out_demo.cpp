#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <iostream>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

class controller {
public
    protected:
    class min_max {
        	double min;
        	double max;
    	};
        std::vector<double> measurements = {};
        std::vector<double> setpoints = {};
        std::vector<double> output = {};
        std::vector<min_max> boundaries;
        bool initialized = false;
    public:
        virtual ~Controller() = default; // to prevent memory leaks from inheritance
        virtual void compute() = 0; // This will hold the logic and return the output
        bool init_system( // initialize values
            const std::vector<double>& new_measurements,
            const std::vector<double>& new_setpoints,
            const std::vector<min_max>& new_boundaries
            ) {
            if (new_boundaries.size() != new_setpoints.size()) return false; // dimensions must match
            for (size_t i{};i < new_setpoints.size(); i++) {
                
            }
            // Other checks tbd...
            // Set new values:
            measurements = new_measurements;
            setpoints = new_setpoints;
            boundaries = new_boundaries;
            initialized = true;
            return true;
        }
        bool update_system( // update measurements
            const std::vector<double>& new_measurements
            ) {
            if (initialized // must be initialized
            // || new_measurements.size() != setpoints.size() // dimensions must match
            ) return false; 
            // Other checks tbd...
            // Set new values:
            measurements = new_measurements;
            return true;    
        }
        double clamp(double val, Bound b) {
        return std::max(b.min, std::min(val, b.max));
        }
};

// using namespace std::chrono_literals;

class in_out_demo : public rclcpp::Node {
	public:
		using GripperCommand = control_msgs::action::GripperCommand;
		
		in_out_demo() : Node("arm_gripper_loop_controller") {
	
			// We publish values to ForwardCommandController // OUTPUT
			forward_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
				"/forward_position_controller/commands",
				10
			);
			
			// We subscribe to read current joint states // MEASUREMENTS
			joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
				"/joint_states",
				10,
				std::bind(&in_out_demo::jointStateCallback, 
					this,
					std::placeholders::_1
				)
			);
			
			// A node that we read the setpoints from // SETPOINTS
				
			// Action client for the gripper
			gripper_client_ = rclcpp_action::create_client<GripperCommand>(
				this,
				"/gripper_action_controller/gripper_cmd"
			);

			// Define target and home positions
			target_pos_ = {1.345, -1.23, 0.264, -0.296, 0.389, -1.5};
			home_pos_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

			// Timer to trigger the control loop
			timer_ = this->create_wall_timer(
				100ms,
				std::bind(&in_out_demo::controlLoopCallback, this)
			);
		}

	private:
		void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
			for (size_t i = 0; i < msg->name.size(); ++i) {
				current_joint_positions_[msg->name[i]] = msg->position[i];
			}
		}

		void sendForwardCommand(const std::vector<double>& positions) {
			auto msg = std_msgs::msg::Float64MultiArray();
			msg.data = positions;
			forward_pub_->publish(msg);
			RCLCPP_INFO(this->get_logger(), "Published forward command.");
		}

		void sendGripperCommand(double position) {
			auto goal_msg = GripperCommand::Goal();
			goal_msg.command.position = position;
			goal_msg.command.max_effort = 5.0;

			auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
			gripper_client_->async_send_goal(goal_msg, send_goal_options);
		}
		
		// We create a new controller class
		// We set up a compute that puts values into the output vector

		void controlLoopCallback() {
			// we call jointStateCallBack // need to make sure measurements are in correct order
			// We call a new functions which listens to the setpoint Node // ToDo where to min and max come from
			// We call our new controller class
			// We call controller.init_system(measurements, setpoints, boundaries/minmax);
			// We call controller.compute(); 
			// We call sendForwardCommand(controller.getOutputs());
			// loop, figure out how to make this into a state machine, we need this to be a discrete controller	
		
			RCLCPP_INFO(this->get_logger(), "Commanding target position...");
			sendForwardCommand(target_pos_);
			std::this_thread::sleep_for(2500ms); 

			RCLCPP_INFO(this->get_logger(), "Closing gripper...");
			sendGripperCommand(-0.7);
			std::this_thread::sleep_for(500ms);

			RCLCPP_INFO(this->get_logger(), "Commanding home position...");
			sendForwardCommand(home_pos_);
			std::this_thread::sleep_for(2500ms); // we need to change to state machinne programmi

			RCLCPP_INFO(this->get_logger(), "Opening gripper...");
			sendGripperCommand(0.0);
			std::this_thread::sleep_for(1500ms); 
		}

		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_pub_;
		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
		rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
		rclcpp::TimerBase::SharedPtr timer_;

		std::map<std::string, double> current_joint_positions_;
		std::vector<double> target_pos_;
		std::vector<double> home_pos_;
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

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// MoveIt 2 Headers for native Kinematics
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

using std::placeholders::_1;

class IKStreamer : public rclcpp::Node {
public:
    IKStreamer(const rclcpp::NodeOptions & options) : Node("ik_streamer_node", options) {
        // 1. Publisher: Sends the calculated joint angles to your PID/Secant nodes
        joint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/controller/joint_setpoints", 10);

        // 2. Subscriber: Listens for X, Y, Z, Pitch, Roll, Yaw targets
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/end_effector_target", 10, std::bind(&IKStreamer::pose_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "IK Streamer initialized. Loading MoveIt Robot Model...");
    }

    void load_robot_model() {
        // Load the URDF geometry from the ROS 2 parameter server
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
            this->shared_from_this(), "robot_description");
        
        kinematic_model_ = robot_model_loader_->getModel();
        
        if (!kinematic_model_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model. Ensure robot_description is published!");
            return;
        }

        kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
        
        // Elephant Robotics typically names the 6-DOF chain "arm_group"
        joint_model_group_ = kinematic_model_->getJointModelGroup("Arm");
	// --- ADD THIS TO FIND YOUR ZERO POSE ---
        
        // 1. Force the mathematical model to the absolute zero position for all joints
        std::vector<double> zero_joints(joint_model_group_->getVariableCount(), 0.0);
        kinematic_state_->setJointGroupPositions(joint_model_group_, zero_joints);
        kinematic_state_->update(); // Force MoveIt to calculate the forward kinematics

        // 2. Get the 6D pose of the tip link 
        // NOTE: Change "link6_flange" to whatever the actual Tip Link of your arm_group is!
        const Eigen::Isometry3d& end_effector_state = kinematic_state_->getGlobalLinkTransform("link6_flange");

        // 3. Extract Translation (XYZ)
        Eigen::Vector3d translation = end_effector_state.translation();
        
        // 4. Extract Rotation (Quaternion for geometry_msgs::Pose)
        Eigen::Quaterniond quaternion(end_effector_state.linear());

        // 5. Print the exact values you need to send to your subscriber
        RCLCPP_INFO(this->get_logger(), "=== THE ZERO POSE FOR THIS ROBOT ===");
        RCLCPP_INFO(this->get_logger(), "Position -> X: %f, Y: %f, Z: %f", 
                    translation.x(), translation.y(), translation.z());
        RCLCPP_INFO(this->get_logger(), "Orientation (Quaternion) -> X: %f, Y: %f, Z: %f, W: %f", 
                    quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
        RCLCPP_INFO(this->get_logger(), "====================================");
        RCLCPP_INFO(this->get_logger(), "MoveIt Kinematics Loaded Successfully! Ready for Cartesian targets.");
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        if (!kinematic_state_ || !joint_model_group_) return;

        // Attempt to calculate the Inverse Kinematics (0.1 second timeout)
        const double timeout = 0.1;
        bool found_ik = kinematic_state_->setFromIK(joint_model_group_, *msg, timeout);

        if (found_ik) {
            std::vector<double> joint_values;
            kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);

            // Package and stream the 6 joint angles to the controller
            auto joint_msg = std_msgs::msg::Float64MultiArray();
            joint_msg.data = joint_values;
            joint_pub_->publish(joint_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "IK Found! Streaming angles to controller...");
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "IK Solution NOT FOUND for this pose (Out of reach or singularity).");
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;

    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // We must allow the node to read the 'robot_description' from launch files
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    
    auto node = std::make_shared<IKStreamer>(options);
    node->load_robot_model(); // Must be called after node is built
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

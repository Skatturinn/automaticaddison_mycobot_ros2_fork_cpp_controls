# Author: el3403
# Function: This script implements a ROS2 Data Logger node.
# Input: Subscriptions to /joint_states, /controller/joint_setpoints, /forward_position_controller/commands, and /end_effector_target.
# Output: A CSV file ("experiment_data/raw_results.csv") containing synchronized time-series data of robot states and control signals.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import csv
import os

class DataLogger(Node):
    """
    Author: el3403
    Pseudo-code:
    1. Initialize a ROS2 node named 'data_logger'.
    2. Create a directory and a CSV file with headers for timestamp, Cartesian targets, and joint data (actual, target, control).
    3. Subscribe to state and command topics to update local buffers.
    4. Set a timer to trigger at 50Hz (0.02s).
    5. On timer trigger, aggregate the most recent data from buffers into a single row and write to the CSV.
    """
    def __init__(self):
        super().__init__('data_logger')
        
        self.filename = "experiment_data/raw_results.csv"
        os.makedirs("experiment_data", exist_ok=True)
        self.csv_file = open(self.filename, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        
        # Create a clean header: Timestamp, Cartesian targets, then all 6 joints
        headers = ['timestamp', 'target_x', 'target_y', 'target_z']
        for i in range(1, 7):
            headers.extend([f'actual_j{i}', f'target_j{i}', f'control_u_j{i}'])
        self.writer.writerow(headers)

        # Initialize placeholders
        self.current_pose = [0.0, 0.0, 0.0]
        self.current_actual = [0.0] * 6
        self.current_target = [0.0] * 6
        self.current_u = [0.0] * 6

        # Subscriptions
        # Author: el3403 - Mapping external sensor and controller data to internal attributes
        self.create_subscription(JointState, '/joint_states', self.actual_cb, 10)
        self.create_subscription(Float64MultiArray, '/controller/joint_setpoints', self.target_cb, 10)
        self.create_subscription(Float64MultiArray, '/forward_position_controller/commands', self.u_cb, 10)
        self.create_subscription(Pose, '/end_effector_target', self.pose_cb, 10)

        # Logging Timer (50Hz to match target sampling time)
        self.create_timer(0.02, self.log_to_csv)
        self.get_logger().info(f"Logging data to {self.filename}")

    # Author: el3403
    # Function: Callback to update actual joint positions
    # Input: JointState message from /joint_states
    # Output: Updated self.current_actual list
    def actual_cb(self, msg):
        self.current_actual = msg.position

    # Author: el3403
    # Function: Callback to update target joint setpoints
    # Input: Float64MultiArray message from /controller/joint_setpoints
    # Output: Updated self.current_target list
    def target_cb(self, msg):
        self.current_target = msg.data

    # Author: el3403
    # Function: Callback to update control signals (u)
    # Input: Float64MultiArray message from /forward_position_controller/commands
    # Output: Updated self.current_u list
    def u_cb(self, msg):
        self.current_u = msg.data

    # Author: el3403
    # Function: Callback to update Cartesian end-effector targets
    # Input: Pose message from /end_effector_target
    # Output: Updated self.current_pose list (X, Y, Z)
    def pose_cb(self, msg):
        self.current_pose = [msg.position.x, msg.position.y, msg.position.z]

    # Author: el3403
    # Function: Periodic function to save data to disk
    # Pseudo-code:
    #   1. Capture current ROS clock time.
    #   2. Construct a list containing [Time, X, Y, Z].
    #   3. Loop through 6 joints and append (Actual, Target, Control) for each.
    #   4. Write the final list as a new row in the CSV file and flush to disk.
    # Input: None (reads from internal attributes)
    # Output: New entry in CSV file
    def log_to_csv(self):
        t = self.get_clock().now().nanoseconds / 1e9
        if t > 0: 
            # Start row with time and Cartesian X, Y, Z
            row = [t, self.current_pose[0], self.current_pose[1], self.current_pose[2]]
            
            # Flatten the joint arrays into the rest of the row
            for i in range(6):
                a = self.current_actual[i] if i < len(self.current_actual) else 0.0
                tgt = self.current_target[i] if i < len(self.current_target) else 0.0
                u = self.current_u[i] if i < len(self.current_u) else 0.0
                row.extend([a, tgt, u])
                
            self.writer.writerow(row)
            self.csv_file.flush()

def main():
    rclpy.init()
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

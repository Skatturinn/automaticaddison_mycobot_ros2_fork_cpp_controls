import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import csv
import os

class DataLogger(Node):
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
        self.create_subscription(JointState, '/joint_states', self.actual_cb, 10)
        self.create_subscription(Float64MultiArray, '/controller/joint_setpoints', self.target_cb, 10)
        self.create_subscription(Float64MultiArray, '/forward_position_controller/commands', self.u_cb, 10)
        self.create_subscription(Pose, '/end_effector_target', self.pose_cb, 10)

        # Logging Timer (50Hz to match your target sampling time)
        self.create_timer(0.02, self.log_to_csv)
        self.get_logger().info(f"Logging data to {self.filename}")

    def actual_cb(self, msg):
        self.current_actual = msg.position

    def target_cb(self, msg):
        self.current_target = msg.data

    def u_cb(self, msg):
        self.current_u = msg.data

    def pose_cb(self, msg):
        self.current_pose = [msg.position.x, msg.position.y, msg.position.z]

    def log_to_csv(self):
        t = self.get_clock().now().nanoseconds / 1e9
        if t > 0: 
            # Start row with time and Cartesian X, Y, Z
            row = [t, self.current_pose, self.current_pose[1], self.current_pose[2]]
            
            # Flatten the joint arrays into the rest of the row
            for i in range(6):
                # Ensure we don't index out of bounds if a message is short
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

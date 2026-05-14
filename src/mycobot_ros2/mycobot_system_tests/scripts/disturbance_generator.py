import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class DisturbanceGenerator(Node):
    def __init__(self):
        super().__init__('disturbance_generator')
        
        # Publisher to the new base controller
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/base_velocity_controller/commands', 
            10
        )
        
        # 50Hz timer
        self.timer = self.create_timer(0.02, self.publish_disturbance)
        self.start_time = None
        self.get_logger().info("Starting Sinusoidal Base Disturbance...")

    def publish_disturbance(self):
        t = self.get_clock().now().nanoseconds / 1e9
        if t == 0:
            return # Wait for Gazebo
            
        if self.start_time is None:
            self.start_time = t
            
        elapsed = t - self.start_time
        
        # Create a sinusoidal velocity disturbance (e.g., Amplitude 0.1 m/s, Freq 0.2 Hz)
        v_base = 0.1 * math.sin(2.0 * math.pi * 0.2 * elapsed)
        
        msg = Float64MultiArray()
        msg.data = [v_base]
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = DisturbanceGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send a final zero command to stop the base
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0]
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

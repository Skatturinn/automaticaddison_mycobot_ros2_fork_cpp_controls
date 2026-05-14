import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.publisher_ = self.create_publisher(Pose, '/end_effector_target', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # ZERO POSE (Matches joint state)
        self.z_x, self.z_y, self.z_z = 0.060620, 0.045600, 0.410140
        
        # SAFE BENT POSE (Experiment Start)
        self.s_x, self.s_y, self.s_z = 0.060620, 0.045600, 0.410140
        self.q_x, self.q_y, self.q_z, self.q_w = -0.707107, 0.0, 0.0, 0.707107
        
        self.start_time = None 
        
        # State Machine Timings
        self.t_move_in = 3.0
        self.t_experiment = 10.0
        self.t_move_out = 3.0
        self.t_total = self.t_move_in + self.t_experiment + self.t_move_out

    def timer_callback(self):
        if self.start_time is None:
            if self.get_clock().now().nanoseconds == 0:
                return # Still waiting for network sync...
            
            self.start_time = self.get_clock().now()
            self.get_logger().info('Clock synced! Starting 16-second trajectory sequence.')
            return

        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9
        
        msg = Pose()
        msg.orientation.x = self.q_x
        msg.orientation.y = self.q_y
        msg.orientation.z = self.q_z
        msg.orientation.w = self.q_w
        
        if t <= self.t_move_in:
            # PHASE 1: Smoothly interpolate from Zero to Safe
            progress = t / self.t_move_in
            smooth_p = (1 - math.cos(progress * math.pi)) / 2.0
            
            msg.position.x = self.z_x + (self.s_x - self.z_x) * smooth_p
            msg.position.y = self.z_y + (self.s_y - self.z_y) * smooth_p
            msg.position.z = self.z_z + (self.s_z - self.z_z) * smooth_p
            
        elif t <= (self.t_move_in + self.t_experiment):
            # PHASE 2: Execute Sine Wave
            t_exp = t - self.t_move_in
            amplitude = 0.04
            offset = self.s_z - amplitude
            z_target = offset + amplitude * math.cos(2 * math.pi * (1/self.t_experiment) * t_exp)
            
            msg.position.x = self.s_x
            msg.position.y = self.s_y
            msg.position.z = z_target
            
        elif t <= self.t_total:
            # PHASE 3: Smoothly interpolate from Safe back to Zero
            t_out = t - (self.t_move_in + self.t_experiment)
            progress = t_out / self.t_move_out
            smooth_p = (1 - math.cos(progress * math.pi)) / 2.0
            
            msg.position.x = self.s_x + (self.z_x - self.s_x) * smooth_p
            msg.position.y = self.s_y + (self.z_y - self.s_y) * smooth_p
            msg.position.z = self.s_z + (self.z_z - self.s_z) * smooth_p
            
        else:
            self.get_logger().info('Trajectory and Reset Complete! Shutting down node.')
            self.timer.cancel()
            raise SystemExit
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

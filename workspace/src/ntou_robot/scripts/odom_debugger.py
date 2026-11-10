#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomDebugger(Node):
    def __init__(self):
        super().__init__('odom_debugger')
        
        self.start_raw = None
        self.start_ekf = None
        
        # Subscribe to Raw Wheel Controller Odometry
        self.sub_raw = self.create_subscription(
            Odometry, '/ackermann_steering_controller/odometry', self.raw_cb, 10)
        
        # Subscribe to EKF Filtered Odometry
        self.sub_ekf = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_cb, 10)
            
        self.get_logger().info("\n--- Debugger Active ---")
        self.get_logger().info("Drive the robot in Gazebo. Compare these numbers to the physical Gazebo grid (1 square = 1 meter).\n")

    def calculate_distance(self, start_msg, current_msg):
        dx = current_msg.pose.pose.position.x - start_msg.pose.pose.position.x
        dy = current_msg.pose.pose.position.y - start_msg.pose.pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def raw_cb(self, msg):
        if self.start_raw is None:
            self.start_raw = msg
            return
        
        # Only print occasionally to avoid terminal spam
        if int(msg.header.stamp.nanosec % 10) == 0:
            dist = self.calculate_distance(self.start_raw, msg)
            self.get_logger().info(f"WHEEL ENCODER claims distance: {dist:.2f} meters")

    def ekf_cb(self, msg):
        if self.start_ekf is None:
            self.start_ekf = msg
            return
            
        if int(msg.header.stamp.nanosec % 10) == 0:
            dist = self.calculate_distance(self.start_ekf, msg)
            self.get_logger().info(f"EKF (Wheels+IMU) claims distance: {dist:.2f} meters")

def main():
    rclpy.init()
    node = OdomDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

class SyncDebugger(Node):
    def __init__(self):
        super().__init__('sync_debugger')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        # State variables
        self.v_cmd = 0.0
        self.v_truth = 0.0
        self.v_wheel = 0.0
        self.v_ekf = 0.0
        self.last_tf_x = None
        self.last_tf_y = None
        self.v_rviz = 0.0

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(TwistStamped, '/ackermann_steering_controller/reference', self.cmd_stamped_cb, 10)
        self.create_subscription(Odometry, '/ground_truth/odometry', self.truth_cb, 10)
        self.create_subscription(Odometry, '/ackermann_steering_controller/odometry', self.wheel_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 10)

        # TF Listener (To track exactly what RViz is drawing via map -> base_footprint)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 10Hz Timer for high-speed tracking
        self.create_timer(0.1, self.report)
        self.get_logger().info('--- High-Speed Sync Debugger Active ---')
        self.get_logger().info('Waiting for data... Drive forward, then STOP, and watch the velocities.')
        self.get_logger().info('FORMAT: [CMD] | [GAZEBO] | [WHEELS] | [EKF] | [RVIZ_TF]')

    def cmd_cb(self, msg): self.v_cmd = msg.linear.x
    def cmd_stamped_cb(self, msg): self.v_cmd = msg.twist.linear.x
    def truth_cb(self, msg): self.v_truth = msg.twist.twist.linear.x
    def wheel_cb(self, msg): self.v_wheel = msg.twist.twist.linear.x
    def ekf_cb(self, msg): self.v_ekf = msg.twist.twist.linear.x

    def report(self):
        # Calculate RViz movement via TF tree (map -> base_footprint)
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            if self.last_tf_x is not None:
                dist = math.hypot(x - self.last_tf_x, y - self.last_tf_y)
                self.v_rviz = dist / 0.1  # distance / time (10Hz = 0.1s)
            
            self.last_tf_x = x
            self.last_tf_y = y
        except TransformException:
            self.v_rviz = -1.0 # Error/Not Ready

        # Format and Print
        def fmt(val): return f"{val:5.2f}"
        
        log = (f"CMD: {fmt(self.v_cmd)} | "
               f"GAZEBO: {fmt(self.v_truth)} | "
               f"WHEELS: {fmt(self.v_wheel)} | "
               f"EKF: {fmt(self.v_ekf)} | "
               f"RVIZ: {fmt(self.v_rviz)}")
        
        self.get_logger().info(log)

def main():
    rclpy.init()
    node = SyncDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

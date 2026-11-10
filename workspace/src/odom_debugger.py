#!/usr/bin/env python3
import math
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

WHEEL_TOPIC = '/ackermann_steering_controller/odometry'
EKF_TOPIC = '/odometry/filtered'
TRUTH_TOPIC = '/ground_truth/odometry'
MAP_TOPIC = '/map'

class OdomDebugger(Node):
    def __init__(self):
        super().__init__('odom_debugger')
        
        # FORCE SIM TIME internally so raw python3 execution syncs with Gazebo
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.lock = threading.Lock()
        
        self.starts = {'wheel': None, 'ekf': None, 'truth': None}
        self.last_msg = {'wheel': None, 'ekf': None, 'truth': None}
        self.prev_msg = {'wheel': None, 'ekf': None, 'truth': None}
        self.map_stamp = None

        self.create_subscription(Odometry, WHEEL_TOPIC, lambda m: self._cb('wheel', m), 10)
        self.create_subscription(Odometry, EKF_TOPIC, lambda m: self._cb('ekf', m), 10)
        self.create_subscription(Odometry, TRUTH_TOPIC, lambda m: self._cb('truth', m), 10)
        # QoS 1 for the map since OccupancyGrid messages are massive
        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._cb_map, map_qos)

        self.create_timer(1.0, self.report)
        self.get_logger().info('Enhanced Debugger + SLAM Tracking Active')

    @staticmethod
    def _xy(msg):
        p = msg.pose.pose.position
        return p.x, p.y

    def _cb(self, key, msg):
        with self.lock:
            if self.starts[key] is None:
                self.starts[key] = msg
            self.prev_msg[key] = self.last_msg[key]
            self.last_msg[key] = msg

    def _cb_map(self, msg):
        with self.lock:
            self.map_stamp = msg.header.stamp

    def reset(self):
        with self.lock:
            for key in self.last_msg:
                self.starts[key] = self.last_msg[key]
        self.get_logger().info('--- REFERENCE RESET ---')

    def _calc_metrics(self, key, now_time):
        start = self.starts[key]
        current = self.last_msg[key]
        prev = self.prev_msg[key]
        
        if not current or not start:
            return None, None, None

        x0, y0 = self._xy(start)
        x1, y1 = self._xy(current)
        dist = math.hypot(x1 - x0, y1 - y0)

        msg_time = current.header.stamp.sec + (current.header.stamp.nanosec * 1e-9)
        lag_ms = (now_time - msg_time) * 1000.0

        vel = 0.0
        if prev:
            px, py = self._xy(prev)
            p_time = prev.header.stamp.sec + (prev.header.stamp.nanosec * 1e-9)
            dt = msg_time - p_time
            if dt > 0.001:
                vel = math.hypot(x1 - px, y1 - py) / dt

        return dist, lag_ms, vel

    def report(self):
        with self.lock:
            now = self.get_clock().now()
            now_time = now.seconds_nanoseconds()[0] + (now.seconds_nanoseconds()[1] * 1e-9)
            
            t_dist, t_lag, t_vel = self._calc_metrics('truth', now_time)
            w_dist, w_lag, w_vel = self._calc_metrics('wheel', now_time)
            e_dist, e_lag, e_vel = self._calc_metrics('ekf', now_time)
            
            map_lag_s = "N/A"
            if self.map_stamp:
                m_time = self.map_stamp.sec + (self.map_stamp.nanosec * 1e-9)
                map_lag_s = f"{(now_time - m_time) * 1000.0:4.0f}ms"

        if t_dist is None:
            return

        def fmt(dist, lag, vel):
            if dist is None: return "N/A"
            ratio = f"{dist/t_dist:.2f}" if t_dist > 0.01 else "-"
            return f"D:{dist:5.2f}m (x{ratio:4}) | V:{vel:4.2f}m/s | Lag:{lag:4.0f}ms"

        self.get_logger().info(f"\nTRUTH -> {fmt(t_dist, t_lag, t_vel)}")
        self.get_logger().info(f"WHEEL -> {fmt(w_dist, w_lag, w_vel)}")
        self.get_logger().info(f"EKF   -> {fmt(e_dist, e_lag, e_vel)}")
        self.get_logger().info(f"SLAM MAP LATENCY -> {map_lag_s}")
        self.get_logger().info("-" * 50)

def _stdin_loop(node):
    for line in sys.stdin:
        node.reset()

def main():
    rclpy.init()
    node = OdomDebugger()
    threading.Thread(target=_stdin_loop, args=(node,), daemon=True).start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

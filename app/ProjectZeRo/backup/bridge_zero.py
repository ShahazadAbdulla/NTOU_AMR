#!/usr/bin/env python3
import json
import math
import struct
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64
import zenoh

# --- PRECOMPILED WIRE FORMATS (LITTLE-ENDIAN, NO PADDING) ---
IMU_V1_STRUCT = struct.Struct("<qB6f")        # 33 bytes: ts_ns, mode, v0..v5
IMU_V2_STRUCT = struct.Struct("<qqBB6f")      # 42 bytes: ts_gyro_ns, ts_accel_ns, mode, flags, v0..v5
GPS_STRUCT = struct.Struct("<q3d4f")          # 48 bytes: ts_ns, lat, lon, alt, speed, acc, bearing, bearing_acc
CLOCK_QUERY_STRUCT = struct.Struct("<qqqi")   # 28 bytes: t1_ns, boot_id, seq, want
CLOCK_REPLY_STRUCT = struct.Struct("<qqqqqq") # 48 bytes: t1_ns, t2_ns, t3_ns, t4_ns, host_rt_ns, host_boot_ns

# Optional phone->host burst report so the bridge can use RTT-filtered offset directly:
# <qqqqd : boot_id, seq, theta_hat_ns, delta_min_ns, quality
CLOCK_REPORT_STRUCT = struct.Struct("<qqqq")  # 32 bytes: boot_id, seq, theta_hat_ns, delta_min_ns

assert IMU_V1_STRUCT.size == 33
assert IMU_V2_STRUCT.size == 42
assert GPS_STRUCT.size == 48
assert CLOCK_QUERY_STRUCT.size == 28
assert CLOCK_REPLY_STRUCT.size == 48


def boottime_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


def realtime_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_REALTIME)


def to_ros_time(stamp_ns: int) -> Time:
    sec, nsec = divmod(int(stamp_ns), 1_000_000_000)
    assert 0 <= nsec < 1_000_000_000, f"Invalid nanosec: {nsec}"
    assert -2147483648 <= sec <= 2147483647, f"sec out of int32 range: {sec}"
    msg = Time()
    msg.sec = int(sec)
    msg.nanosec = int(nsec)
    return msg


class ClockSyncEstimator:
    """
    Maps Phone CLOCK_BOOTTIME -> Host CLOCK_BOOTTIME:
        theta(t) = theta_0 + s * (t - t_ref)
    Supports both two-way burst filtering (RFC 5905 + Marzullo) and one-way min-delay fallback/cross-check.
    """

    def __init__(self, skew_window_s: float = 600.0, max_slew_ppm: float = 500.0):
        self.skew_window_ns = int(skew_window_s * 1e9)
        self.max_slew_rate = max_slew_ppm * 1e-6  # ns per ns
        self.lock = threading.Lock()

        self.boot_id = None
        self.synced = False
        self.degraded = False

        # Affine model: theta(t_phone) = theta_0 + slope * (t_phone - t_ref_phone)
        self.theta_0 = 0.0
        self.t_ref_phone = 0
        self.slope = 0.0  # dimensionless (s * 1e6 = ppm)

        # Slew state
        self.slew_target_offset = 0.0
        self.last_predict_phone_ns = None

        # History of accepted bursts: deque of (t_phone_ns, theta_hat_ns, delta_min_ns)
        self.history = deque()
        self.one_way_window = deque(maxlen=200)

        # Diagnostics counters
        self.bursts_ok = 0
        self.bursts_failed = 0
        self.consecutive_rejects = 0
        self.last_burst_host_ns = 0
        self.delta_best_ns = None
        self.last_delta_min_ns = 0
        self.last_delta_p99_ns = 0
        self.offset_std_ns = 0.0

    def reset(self, new_boot_id=None):
        with self.lock:
            self.boot_id = new_boot_id
            self.synced = False
            self.degraded = False
            self.theta_0 = 0.0
            self.t_ref_phone = 0
            self.slope = 0.0
            self.slew_target_offset = 0.0
            self.last_predict_phone_ns = None
            self.history.clear()
            self.one_way_window.clear()
            self.consecutive_rejects = 0
            self.delta_best_ns = None

    @staticmethod
    def marzullo_agreement(samples):
        """
        samples: list of (t_phone, theta_i, delta_i)
        Returns True if a majority of intervals [theta_i - delta_i/2, theta_i + delta_i/2] intersect.
        """
        if not samples:
            return False
        events = []
        for _, theta, delta in samples:
            half = delta / 2.0
            events.append((theta - half, -1))  # interval start
            events.append((theta + half, +1))  # interval end
        events.sort(key=lambda x: (x[0], x[1]))

        count = 0
        max_overlap = 0
        for _, etype in events:
            if etype == -1:
                count += 1
                if count > max_overlap:
                    max_overlap = count
            else:
                count -= 1
        return max_overlap > (len(samples) // 2)

    def process_burst(self, raw_exchanges, boot_id: int) -> bool:
        """
        raw_exchanges: list of (t1, t2, t3, t4) all in CLOCK_BOOTTIME ns.
        """
        if self.boot_id is not None and boot_id != self.boot_id:
            self.reset(new_boot_id=boot_id)
        elif self.boot_id is None:
            self.boot_id = boot_id

        valid = []
        for t1, t2, t3, t4 in raw_exchanges:
            delta = (t4 - t1) - (t3 - t2)
            if delta < 0 or t4 <= t1 or t3 < t2:
                continue
            theta = 0.5 * ((t2 - t1) + (t3 - t4))
            valid.append((t1, theta, delta))

        with self.lock:
            if len(valid) < (len(raw_exchanges) // 2) + 1:
                self.bursts_failed += 1
                self.degraded = True
                return False

            if not self.marzullo_agreement(valid):
                self.bursts_failed += 1
                self.degraded = True
                return False

            # Sort by delta_i ascending (min-delay selection)
            valid.sort(key=lambda x: x[2])
            t_best, theta_hat, delta_min = valid[0]
            deltas_sorted = [x[2] for x in valid]
            p99_idx = min(len(deltas_sorted) - 1, int(math.ceil(0.99 * len(deltas_sorted))) - 1)
            self.last_delta_min_ns = int(delta_min)
            self.last_delta_p99_ns = int(deltas_sorted[p99_idx])

            return self._ingest_estimate_locked(t_best, theta_hat, delta_min)

    def ingest_phone_burst_report(self, t_phone_ns: int, theta_hat: float, delta_min: float, boot_id: int) -> bool:
        if self.boot_id is not None and boot_id != self.boot_id:
            self.reset(new_boot_id=boot_id)
        elif self.boot_id is None:
            self.boot_id = boot_id

        with self.lock:
            self.last_delta_min_ns = int(delta_min)
            return self._ingest_estimate_locked(t_phone_ns, theta_hat, delta_min)

    def _ingest_estimate_locked(self, t_phone_ns: int, theta_hat: float, delta_min: float) -> bool:
        now_host = boottime_ns()

        # First valid sync: step immediately
        if not self.synced:
            self.t_ref_phone = t_phone_ns
            self.theta_0 = float(theta_hat)
            self.slew_target_offset = 0.0
            self.slope = 0.0
            self.delta_best_ns = delta_min
            self.history.append((t_phone_ns, float(theta_hat), float(delta_min)))
            self.synced = True
            self.degraded = False
            self.bursts_ok += 1
            self.last_burst_host_ns = now_host
            return True

        # Outlier gate
        if self.delta_best_ns is None or delta_min < self.delta_best_ns:
            self.delta_best_ns = delta_min

        theta_pred = self.theta_0 + self.slope * (t_phone_ns - self.t_ref_phone) + self.slew_target_offset
        residual = theta_hat - theta_pred

        delay_gate = self.delta_best_ns + max(1_000_000.0, 0.5 * self.delta_best_ns)
        sigma_gate = max(2_000_000.0, 5.0 * self.offset_std_ns) if self.offset_std_ns > 0 else 5_000_000.0

        if delta_min > delay_gate or abs(residual) > sigma_gate:
            self.consecutive_rejects += 1
            if self.consecutive_rejects < 3:
                self.bursts_failed += 1
                self.degraded = True
                return False
            # 3 consecutive rejects -> accept real step
            self.consecutive_rejects = 0
            self.t_ref_phone = t_phone_ns
            self.theta_0 = float(theta_hat)
            self.slew_target_offset = 0.0
        else:
            self.consecutive_rejects = 0
            # Step vs Slew policy (§10.4)
            abs_res = abs(residual)
            if abs_res < 100_000.0:  # < 100 us: ignore
                pass
            elif abs_res <= 1_000_000.0:  # 100 us .. 1 ms: slew
                self.slew_target_offset += residual
            else:
                # > 1 ms: require confirmation or step if persistent
                self.slew_target_offset += residual

        # Maintain sliding window for least-squares skew regression
        self.history.append((t_phone_ns, float(theta_hat), float(delta_min)))
        cutoff = t_phone_ns - self.skew_window_ns
        while len(self.history) > 2 and self.history[0][0] < cutoff:
            self.history.popleft()

        if len(self.history) >= 3:
            self._fit_skew_locked()

        self.bursts_ok += 1
        self.degraded = False
        self.last_burst_host_ns = now_host
        return True

    def _fit_skew_locked(self):
        # Ordinary least squares over (t_i - t_ref, theta_i)
        n = len(self.history)
        t0 = self.history[0][0]
        xs = [(h[0] - t0) * 1e-9 for h in self.history]  # seconds
        ys = [h[1] for h in self.history]                 # ns
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        denom = sum((x - mean_x) ** 2 for x in xs)
        if denom > 1.0:  # at least ~1s span
            slope_ns_per_s = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom
            # Clamp unreasonable oscillator skew to +/- 200 ppm
            slope_dimensionless = max(-200e-6, min(200e-6, slope_ns_per_s * 1e-9))
            self.slope = slope_dimensionless
            residuals = [y - (mean_y + slope_ns_per_s * (x - mean_x)) for x, y in zip(xs, ys)]
            self.offset_std_ns = math.sqrt(sum(r * r for r in residuals) / n)

    def update_one_way(self, ts_phone_ns: int, arrival_host_boot_ns: int):
        with self.lock:
            self.one_way_window.append(arrival_host_boot_ns - ts_phone_ns)

    def predict_host_boot_ns(self, ts_phone_ns: int, arrival_host_boot_ns: int) -> tuple[int, bool]:
        with self.lock:
            if not self.synced:
                # Fallback: one-way minimum delay estimate
                self.one_way_window.append(arrival_host_boot_ns - ts_phone_ns)
                min_one_way = min(self.one_way_window)
                return int(ts_phone_ns + min_one_way), False

            # Apply bounded slewing (max 500 ppm)
            if self.last_predict_phone_ns is not None and ts_phone_ns > self.last_predict_phone_ns:
                dt = ts_phone_ns - self.last_predict_phone_ns
                max_step = self.max_slew_rate * dt
                if abs(self.slew_target_offset) <= max_step:
                    self.theta_0 += self.slew_target_offset
                    self.slew_target_offset = 0.0
                else:
                    step = math.copysign(max_step, self.slew_target_offset)
                    self.theta_0 += step
                    self.slew_target_offset -= step
            self.last_predict_phone_ns = ts_phone_ns

            theta = self.theta_0 + self.slope * (ts_phone_ns - self.t_ref_phone)
            return int(ts_phone_ns + round(theta)), True


class PhoneZenohBridge(Node):
    def __init__(self):
        super().__init__("phone_zenoh_bridge")

        # 1. Declare and validate parameters (§13.2)
        self.declare_parameter("zenoh_endpoint", "tcp/127.0.0.1:7447")
        self.declare_parameter("zenoh_mode", "client")
        self.declare_parameter("listen_fallback", True)
        self.declare_parameter("clock_query_key", "phone/clock/query")
        self.declare_parameter("clock_report_key", "phone/clock/report")
        self.declare_parameter("imu_key", "phone/sensors/imu")
        self.declare_parameter("gps_key", "phone/sensors/gps")
        self.declare_parameter("frame_id", "phone_imu_link")
        self.declare_parameter("max_slew_ppm", 500.0)
        self.declare_parameter("future_clamp_ns", 5_000_000)
        self.declare_parameter("skew_window_s", 600.0)
        self.declare_parameter("sigma_gyro_rad_s", 0.01)
        self.declare_parameter("sigma_accel_m_s2", 0.05)
        self.declare_parameter("allow_test_bias", False)
        self.declare_parameter("test_bias_ns", 0)

        if self.get_parameter("use_sim_time").value:
            raise RuntimeError("FATAL: phone_zenoh_bridge requires use_sim_time=false!")

        self.frame_id = self.get_parameter("frame_id").value
        self.future_clamp_ns = int(self.get_parameter("future_clamp_ns").value)
        sigma_gyro = float(self.get_parameter("sigma_gyro_rad_s").value)
        sigma_accel = float(self.get_parameter("sigma_accel_m_s2").value)
        self.var_gyro = sigma_gyro ** 2
        self.var_accel = sigma_accel ** 2

        allow_bias = bool(self.get_parameter("allow_test_bias").value)
        req_bias = int(self.get_parameter("test_bias_ns").value)
        if req_bias != 0 and not allow_bias:
            raise RuntimeError("test_bias_ns != 0 requires allow_test_bias=true")
        self.test_bias_ns = req_bias if allow_bias else 0

        self.estimator = ClockSyncEstimator(
            skew_window_s=float(self.get_parameter("skew_window_s").value),
            max_slew_ppm=float(self.get_parameter("max_slew_ppm").value),
        )

        # State & Counters (guarded by lock)
        self.state_lock = threading.Lock()
        self.last_imu_stamp_ns = 0
        self.last_k_ns = None
        self.samples_in = 0
        self.samples_out = 0
        self.reordered_count = 0
        self.future_clamped_count = 0
        self.bad_payload_count = 0
        self.clock_steps_detected = 0

        # 2. ROS 2 Publishers with strict QoS (§12)
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        diag_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.imu_pub = self.create_publisher(Imu, "/imu/data", imu_qos)
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", gps_qos)
        self.bearing_pub = self.create_publisher(Float64, "/gps/bearing", gps_qos)
        self.diag_pub = self.create_publisher(DiagnosticStatus, "/phone/clock/status", diag_qos)

        # 3. Open Application-Level Zenoh Session (Auto-Fallback if zenohd is not running)
        endpoint = self.get_parameter("zenoh_endpoint").value
        mode = self.get_parameter("zenoh_mode").value
        listen_fallback = bool(self.get_parameter("listen_fallback").value)

        self.zsession = None
        if mode == "client":
            try:
                zconf = zenoh.Config()
                zconf.insert_json5("mode", '"client"')
                zconf.insert_json5("scouting/multicast/enabled", "false")
                zconf.insert_json5("connect/endpoints", json.dumps([endpoint]))
                self.zsession = zenoh.open(zconf)
                self.get_logger().info(f"Connected to local zenohd router at {endpoint}")
            except Exception as e:
                if not listen_fallback:
                    raise
                self.get_logger().warn(
                    f"No zenohd router at {endpoint} ({e}). "
                    "Falling back to direct PEER listener on tcp/0.0.0.0:7447 & udp/0.0.0.0:7447!"
                )

        if self.zsession is None:
            zconf = zenoh.Config()
            zconf.insert_json5("mode", '"peer"')
            zconf.insert_json5("scouting/multicast/enabled", "false")
            zconf.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447", "udp/0.0.0.0:7447"]')
            self.zsession = zenoh.open(zconf)
            self.get_logger().info("Zenoh session listening directly on tcp/0.0.0.0:7447 and udp/0.0.0.0:7447")

        # 4. Declare Clock Queryable FIRST (§11.1 & §11.2)
        clock_key = self.get_parameter("clock_query_key").value
        self.queryable = self.zsession.declare_queryable(clock_key, self._on_clock_query)

        # Optional listener for phone-computed burst offset reports
        report_key = self.get_parameter("clock_report_key").value
        self.sub_report = self.zsession.declare_subscriber(report_key, self._on_clock_report)

        # 5. Declare Sensor Subscribers
        imu_key = self.get_parameter("imu_key").value
        gps_key = self.get_parameter("gps_key").value
        self.sub_imu = self.zsession.declare_subscriber(imu_key, self._on_imu_sample)
        self.sub_gps = self.zsession.declare_subscriber(gps_key, self._on_gps_sample)

        # 6. 1 Hz Diagnostic Status Timer (§11.5)
        self.diag_timer = self.create_timer(1.0, self._publish_diagnostics)

    # --- ALLOCATION-FREE CLOCK QUERYABLE HANDLER (§11.2) ---
    def _on_clock_query(self, query: zenoh.Query):
        t2 = boottime_ns()
        try:
            payload = query.payload
            if payload is None:
                with self.state_lock:
                    self.bad_payload_count += 1
                return
            raw = bytes(payload)
            if len(raw) != CLOCK_QUERY_STRUCT.size:
                with self.state_lock:
                    self.bad_payload_count += 1
                return
            t1, boot_id, seq, _ = CLOCK_QUERY_STRUCT.unpack(raw)
        except Exception:
            with self.state_lock:
                self.bad_payload_count += 1
            return

        t3 = boottime_ns()
        rt = realtime_ns()
        reply_bytes = CLOCK_REPLY_STRUCT.pack(t1, t2, t3, 0, rt, t3)
        try:
            query.reply(query.key_expr, reply_bytes)
        except Exception:
            pass

    def _on_clock_report(self, sample: zenoh.Sample):
        try:
            raw = bytes(sample.payload)
            if len(raw) != CLOCK_REPORT_STRUCT.size:
                return
            boot_id, seq, theta_hat_ns, delta_min_ns = CLOCK_REPORT_STRUCT.unpack(raw)
            t_phone_approx = boottime_ns() - theta_hat_ns
            self.estimator.ingest_phone_burst_report(t_phone_approx, float(theta_hat_ns), float(delta_min_ns), boot_id)
        except Exception:
            pass

    # --- BOOTTIME TO ROS REALTIME CONVERSION (§10.5) ---
    def _boot_to_ros_ns(self, stamp_host_boot_ns: int) -> tuple[int, int]:
        rt_now = realtime_ns()
        bt_now = boottime_ns()
        k = rt_now - bt_now
        with self.state_lock:
            if self.last_k_ns is not None and abs(k - self.last_k_ns) > 1_000_000:
                self.clock_steps_detected += 1
                self.get_logger().warn(f"clock_step detected on host! delta_k={(k - self.last_k_ns)/1e6:.2f} ms")
            self.last_k_ns = k
        return stamp_host_boot_ns + k + self.test_bias_ns, rt_now

    # --- IMU INGESTION (§11.3) ---
    def _on_imu_sample(self, sample: zenoh.Sample):
        arrival_boot_ns = boottime_ns()
        raw = bytes(sample.payload)
        n = len(raw)

        if n == IMU_V2_STRUCT.size:
            ts_gyro_ns, ts_accel_ns, mode, flags, v0, v1, v2, v3, v4, v5 = IMU_V2_STRUCT.unpack(raw)
            ts_phone = ts_gyro_ns
        elif n == IMU_V1_STRUCT.size:
            ts_phone, mode, v0, v1, v2, v3, v4, v5 = IMU_V1_STRUCT.unpack(raw)
        else:
            with self.state_lock:
                self.bad_payload_count += 1
            return

        self.estimator.update_one_way(ts_phone, arrival_boot_ns)
        stamp_boot_ns, _ = self.estimator.predict_host_boot_ns(ts_phone, arrival_boot_ns)
        stamp_ros_ns, now_ros_ns = self._boot_to_ros_ns(stamp_boot_ns)

        with self.state_lock:
            self.samples_in += 1
            if stamp_ros_ns > now_ros_ns + self.future_clamp_ns:
                stamp_ros_ns = now_ros_ns
                self.future_clamped_count += 1

            if stamp_ros_ns <= self.last_imu_stamp_ns:
                self.reordered_count += 1
                return
            self.last_imu_stamp_ns = stamp_ros_ns
            self.samples_out += 1

        msg = Imu()
        msg.header.stamp = to_ros_time(stamp_ros_ns)
        msg.header.frame_id = self.frame_id

        if mode == 0:  # Fused Quaternion
            msg.orientation.x = float(v0)
            msg.orientation.y = float(v1)
            msg.orientation.z = float(v2)
            msg.orientation.w = float(v3)
            msg.orientation_covariance[0] = 0.01
            msg.orientation_covariance[4] = 0.01
            msg.orientation_covariance[8] = 0.01
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0
        else:          # Raw Linear Accel + Gyro
            msg.orientation_covariance[0] = -1.0
            msg.linear_acceleration.x = float(v0)
            msg.linear_acceleration.y = float(v1)
            msg.linear_acceleration.z = float(v2)
            msg.angular_velocity.x = float(v3)
            msg.angular_velocity.y = float(v4)
            msg.angular_velocity.z = float(v5)
            msg.linear_acceleration_covariance[0] = self.var_accel
            msg.linear_acceleration_covariance[4] = self.var_accel
            msg.linear_acceleration_covariance[8] = self.var_accel
            msg.angular_velocity_covariance[0] = self.var_gyro
            msg.angular_velocity_covariance[4] = self.var_gyro
            msg.angular_velocity_covariance[8] = self.var_gyro

        self.imu_pub.publish(msg)

    # --- GPS INGESTION (§11.4) ---
    def _on_gps_sample(self, sample: zenoh.Sample):
        arrival_boot_ns = boottime_ns()
        raw = bytes(sample.payload)
        if len(raw) != GPS_STRUCT.size:
            with self.state_lock:
                self.bad_payload_count += 1
            return

        ts_phone, lat, lon, alt, speed, acc, bearing, bearing_acc = GPS_STRUCT.unpack(raw)
        stamp_boot_ns, _ = self.estimator.predict_host_boot_ns(ts_phone, arrival_boot_ns)
        stamp_ros_ns, now_ros_ns = self._boot_to_ros_ns(stamp_boot_ns)
        if stamp_ros_ns > now_ros_ns + self.future_clamp_ns:
            stamp_ros_ns = now_ros_ns

        msg = NavSatFix()
        msg.header.stamp = to_ros_time(stamp_ros_ns)
        msg.header.frame_id = "phone_gps_link"
        msg.status.service = NavSatStatus.SERVICE_GPS

        if math.isfinite(acc) and acc > 0.0:
            msg.status.status = NavSatStatus.STATUS_FIX
            var_h = float(acc * acc)
            msg.position_covariance = [
                var_h, 0.0, 0.0,
                0.0, var_h, 0.0,
                0.0, 0.0, var_h * 4.0,
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.position_covariance = [0.0] * 9
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt)
        self.gps_pub.publish(msg)

        if math.isfinite(bearing):
            b_msg = Float64()
            b_msg.data = float(bearing)
            self.bearing_pub.publish(b_msg)

    # --- 1 Hz DIAGNOSTICS (§11.5) ---
    def _publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = "phone_zenoh_bridge:clock_sync"
        status.hardware_id = "samsung_s23_zenoh"

        with self.estimator.lock:
            synced = self.estimator.synced
            degraded = self.estimator.degraded
            theta_ns = int(self.estimator.theta_0)
            std_ns = int(self.estimator.offset_std_ns)
            skew_ppm = self.estimator.slope * 1e6
            d_min = self.estimator.last_delta_min_ns
            d_p99 = self.estimator.last_delta_p99_ns
            b_ok = self.estimator.bursts_ok
            b_fail = self.estimator.bursts_failed
            age_s = (boottime_ns() - self.estimator.last_burst_host_ns) / 1e9 if self.estimator.last_burst_host_ns else -1.0

        with self.state_lock:
            s_in = self.samples_in
            s_out = self.samples_out
            reord = self.reordered_count
            clamped = self.future_clamped_count

        if synced and not degraded:
            sync_state = "SYNCED"
            status.level = DiagnosticStatus.OK
        elif synced and degraded:
            sync_state = "DEGRADED"
            status.level = DiagnosticStatus.WARN
        else:
            sync_state = "UNSYNCED"
            status.level = DiagnosticStatus.WARN

        status.message = sync_state
        kvs = [
            ("sync_state", sync_state),
            ("offset_ns", str(theta_ns)),
            ("offset_std_ns", str(std_ns)),
            ("skew_ppm", f"{skew_ppm:.3f}"),
            ("path_delay_min_ns", str(d_min)),
            ("path_delay_p99_ns", str(d_p99)),
            ("last_burst_age_s", f"{age_s:.1f}"),
            ("bursts_ok", str(b_ok)),
            ("bursts_failed", str(b_fail)),
            ("samples_in", str(s_in)),
            ("samples_out", str(s_out)),
            ("reordered", str(reord)),
            ("future_clamped", str(clamped)),
        ]
        status.values = [KeyValue(key=k, value=v) for k, v in kvs]
        self.diag_pub.publish(status)

    def destroy_node(self):
        try:
            self.zsession.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PhoneZenohBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

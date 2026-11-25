import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from .protocol import ImuSample, GpsSample
from .time_sync import ClockSyncEstimator, boottime_ns


def to_ros_time(stamp_ns: int) -> Time:
    sec, nsec = divmod(int(stamp_ns), 1_000_000_000)
    msg = Time()
    msg.sec = int(sec)
    msg.nanosec = int(nsec)
    return msg


class ProjectZeroRos2Publisher(Node):
    def __init__(
        self,
        node_name: str = "project_zero_bridge",
        imu_topic: str = "/project_zero/imu/data",
        gps_fix_topic: str = "/project_zero/gps/fix",
        gps_bearing_topic: str = "/project_zero/gps/bearing",
        camera_topic: str = "/project_zero/camera/image/compressed",
        clock_status_topic: str = "/project_zero/clock/status",
        imu_frame_id: str = "phone_imu_link",
        gps_frame_id: str = "phone_gps_link",
        camera_frame_id: str = "phone_camera_optical_frame",
        estimator: ClockSyncEstimator = None,
    ):
        super().__init__(node_name)
        self.imu_frame_id = imu_frame_id
        self.gps_frame_id = gps_frame_id
        self.camera_frame_id = camera_frame_id
        self.estimator = estimator or ClockSyncEstimator()
        self.last_imu_stamp_ns = 0
        self.reordered_count = 0
        self.samples_out = 0

        imu_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        gps_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
        cam_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        diag_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.imu_pub = self.create_publisher(Imu, imu_topic, imu_qos)
        self.gps_pub = self.create_publisher(NavSatFix, gps_fix_topic, gps_qos)
        self.bearing_pub = self.create_publisher(Float64, gps_bearing_topic, gps_qos)
        self.cam_pub = self.create_publisher(CompressedImage, camera_topic, cam_qos)
        self.diag_pub = self.create_publisher(DiagnosticStatus, clock_status_topic, diag_qos)

        self.create_timer(1.0, self._publish_clock_status)

    def publish_imu(self, sample: ImuSample, stamp_ros_ns: int):
        if stamp_ros_ns <= self.last_imu_stamp_ns:
            self.reordered_count += 1
            return
        self.last_imu_stamp_ns = stamp_ros_ns
        self.samples_out += 1

        msg = Imu()
        msg.header.stamp = to_ros_time(stamp_ros_ns)
        msg.header.frame_id = self.imu_frame_id

        if sample.mode == 2:
            # Universal Mode: Orientation (Quat) + Gyro + Accel
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (
                sample.qx, sample.qy, sample.qz, sample.qw
            )
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
                sample.gx, sample.gy, sample.gz
            )
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
                sample.ax, sample.ay, sample.az
            )
            msg.orientation_covariance[0] = msg.orientation_covariance[4] = msg.orientation_covariance[8] = 0.01
            msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = msg.angular_velocity_covariance[8] = 0.0001
            msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] = msg.linear_acceleration_covariance[8] = 0.0025
        elif sample.mode == 0:
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (
                sample.qx, sample.qy, sample.qz, sample.qw
            )
            msg.orientation_covariance[0] = msg.orientation_covariance[4] = msg.orientation_covariance[8] = 0.01
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0
        else:
            msg.orientation_covariance[0] = -1.0
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
                sample.ax, sample.ay, sample.az
            )
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
                sample.gx, sample.gy, sample.gz
            )
            msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] = msg.linear_acceleration_covariance[8] = 0.0025
            msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = msg.angular_velocity_covariance[8] = 0.0001

        self.imu_pub.publish(msg)

    def publish_gps(self, sample: GpsSample, stamp_ros_ns: int):
        msg = NavSatFix()
        msg.header.stamp = to_ros_time(stamp_ros_ns)
        msg.header.frame_id = self.gps_frame_id
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude, msg.longitude, msg.altitude = sample.latitude, sample.longitude, sample.altitude

        if math.isfinite(sample.accuracy) and sample.accuracy > 0.0:
            msg.status.status = NavSatStatus.STATUS_FIX
            var_h = float(sample.accuracy ** 2)
            msg.position_covariance = [var_h, 0.0, 0.0, 0.0, var_h, 0.0, 0.0, 0.0, var_h * 4.0]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.position_covariance = [0.0] * 9
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.gps_pub.publish(msg)
        if math.isfinite(sample.bearing):
            b_msg = Float64()
            b_msg.data = float(sample.bearing)
            self.bearing_pub.publish(b_msg)

    def publish_camera(self, jpeg_bytes: bytes, stamp_ros_ns: int):
        msg = CompressedImage()
        msg.header.stamp = to_ros_time(stamp_ros_ns)
        msg.header.frame_id = self.camera_frame_id
        msg.format = "jpeg"
        msg.data = jpeg_bytes
        self.cam_pub.publish(msg)

    def _publish_clock_status(self):
        status = DiagnosticStatus()
        status.name = "project_zero:clock_sync"
        status.hardware_id = "android_zenoh_edge"
        with self.estimator.lock:
            synced = self.estimator.synced
            degraded = self.estimator.degraded
            offset_ns = int(self.estimator.theta_0)
            std_ns = int(self.estimator.offset_std_ns)
            skew_ppm = self.estimator.slope * 1e6
            d_min_ns = int(self.estimator.last_delta_min_ns)
            b_ok = self.estimator.bursts_ok
            b_fail = self.estimator.bursts_failed
            clamped = self.estimator.future_clamped_count

        sync_state = "SYNCED" if (synced and not degraded) else ("DEGRADED" if synced else "UNSYNCED_FALLBACK")
        status.level = DiagnosticStatus.OK if sync_state == "SYNCED" else DiagnosticStatus.WARN
        status.message = sync_state
        status.values = [
            KeyValue(key="sync_state", value=sync_state),
            KeyValue(key="offset_ns", value=str(offset_ns)),
            KeyValue(key="offset_std_ns", value=str(std_ns)),
            KeyValue(key="skew_ppm", value=f"{skew_ppm:.3f}"),
            KeyValue(key="path_delay_min_ns", value=str(d_min_ns)),
            KeyValue(key="bursts_ok", value=str(b_ok)),
            KeyValue(key="bursts_failed", value=str(b_fail)),
            KeyValue(key="samples_out", value=str(self.samples_out)),
            KeyValue(key="reordered", value=str(self.reordered_count)),
            KeyValue(key="future_clamped", value=str(clamped)),
        ]
        self.diag_pub.publish(status)

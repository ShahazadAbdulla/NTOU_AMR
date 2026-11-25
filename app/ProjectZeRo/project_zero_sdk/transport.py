import threading
import socket
import json
import struct
from typing import Callable, Optional
import zenoh

from .protocol import (
    CLOCK_QUERY_STRUCT,
    CLOCK_REPLY_STRUCT,
    CLOCK_REPORT_STRUCT,
    ImuSample,
    GpsSample,
    unpack_imu,
    unpack_gps,
)
from .time_sync import ClockSyncEstimator, boottime_ns, realtime_ns


class ZenohSensorNode:
    """
    Modular Zenoh transport node.
    Supports mode='router' (default standalone server), 'peer', or 'client' (connects to zenohd).
    """

    def __init__(
        self,
        mode: str = "router",
        endpoint: str = "tcp/0.0.0.0:7447",
        fallback_to_router: bool = True,
        imu_key: str = "phone/sensors/imu",
        gps_key: str = "phone/sensors/gps",
        camera_key: str = "phone/sensors/camera",
        clock_query_key: str = "phone/clock/query",
        clock_report_key: str = "phone/clock/report",
        on_imu: Optional[Callable[[ImuSample, int], None]] = None,
        on_gps: Optional[Callable[[GpsSample, int], None]] = None,
        on_camera: Optional[Callable[[bytes, int], None]] = None,
        estimator: Optional[ClockSyncEstimator] = None,
    ):
        self.estimator = estimator or ClockSyncEstimator()
        self.on_imu = on_imu
        self.on_gps = on_gps
        self.on_camera = on_camera

        self.samples_in = 0
        self.bad_payload_count = 0

        self.session = self._open_session(mode, endpoint, fallback_to_router)

        # Always register the lightweight clock handlers
        self._udp_running = True
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp_sock.bind(("0.0.0.0", 7448))
        self._udp_thread = threading.Thread(target=self._udp_clock_loop, daemon=True)
        self._udp_thread.start()

        self._queryable = self.session.declare_queryable(clock_query_key, self._handle_clock_query)
        self._sub_report = self.session.declare_subscriber(clock_report_key, self._handle_clock_report)

        # Only subscribe to streams the user actually wants
        self._sub_imu = self.session.declare_subscriber(imu_key, self._handle_imu) if on_imu else None
        self._sub_gps = self.session.declare_subscriber(gps_key, self._handle_gps) if on_gps else None
        self._sub_cam = self.session.declare_subscriber(camera_key, self._handle_camera) if on_camera else None

    @staticmethod
    def _open_session(mode: str, endpoint: str, fallback_to_router: bool) -> zenoh.Session:
        if mode == "client":
            try:
                conf = zenoh.Config()
                conf.insert_json5("mode", '"client"')
                conf.insert_json5("scouting/multicast/enabled", "false")
                conf.insert_json5("connect/endpoints", json.dumps([endpoint]))
                return zenoh.open(conf)
            except Exception:
                if not fallback_to_router:
                    raise
                mode = "router"
                endpoint = "tcp/0.0.0.0:7447"

        conf = zenoh.Config()
        conf.insert_json5("mode", json.dumps(mode))  # "router" or "peer"
        conf.insert_json5("scouting/multicast/enabled", "false")
        if "0.0.0.0" in endpoint:
            conf.insert_json5("listen/endpoints", json.dumps([endpoint, "udp/0.0.0.0:7447"]))
        else:
            conf.insert_json5("connect/endpoints", json.dumps([endpoint]))
        return zenoh.open(conf)

    def _udp_clock_loop(self):
        while self._udp_running:
            try:
                raw, addr = self._udp_sock.recvfrom(64)
                t2 = boottime_ns()
                if len(raw) == CLOCK_QUERY_STRUCT.size:
                    t1, boot_id, seq, _ = CLOCK_QUERY_STRUCT.unpack(raw)
                    t3 = boottime_ns()
                    rt = realtime_ns()
                    reply = CLOCK_REPLY_STRUCT.pack(t1, t2, t3, 0, rt, t3)
                    self._udp_sock.sendto(reply, addr)
            except Exception:
                if not self._udp_running:
                    break

    def _handle_clock_query(self, query: zenoh.Query):
        t2 = boottime_ns()
        try:
            if query.payload is None:
                return
            raw = bytes(query.payload)
            if len(raw) != CLOCK_QUERY_STRUCT.size:
                return
            t1, boot_id, seq, _ = CLOCK_QUERY_STRUCT.unpack(raw)
        except Exception:
            return
        t3 = boottime_ns()
        rt = realtime_ns()
        reply = CLOCK_REPLY_STRUCT.pack(t1, t2, t3, 0, rt, t3)
        try:
            query.reply(query.key_expr, reply)
        except Exception:
            pass

    def _handle_clock_report(self, sample: zenoh.Sample):
        try:
            raw = bytes(sample.payload)
            if len(raw) == CLOCK_REPORT_STRUCT.size:
                boot_id, t1_best_ns, theta_hat_ns, delta_min_ns = CLOCK_REPORT_STRUCT.unpack(raw)
                self.estimator.ingest_phone_burst_report(
                    t1_best_ns, float(theta_hat_ns), float(delta_min_ns), boot_id
                )
        except Exception:
            pass

    def _handle_imu(self, sample: zenoh.Sample):
        arrival_bt = boottime_ns()
        imu = unpack_imu(bytes(sample.payload))
        if imu is None:
            self.bad_payload_count += 1
            return
        self.samples_in += 1
        self.estimator.update_one_way(imu.ts_gyro_ns, arrival_bt)
        synced_rt_ns = self.estimator.to_host_realtime_ns(imu.ts_gyro_ns, arrival_bt)
        if self.on_imu:
            self.on_imu(imu, synced_rt_ns)

    def _handle_gps(self, sample: zenoh.Sample):
        arrival_bt = boottime_ns()
        gps = unpack_gps(bytes(sample.payload))
        if gps is None:
            self.bad_payload_count += 1
            return
        synced_rt_ns = self.estimator.to_host_realtime_ns(gps.ts_phone_ns, arrival_bt)
        if self.on_gps:
            self.on_gps(gps, synced_rt_ns)

    def _handle_camera(self, sample: zenoh.Sample):
        arrival_bt = boottime_ns()
        payload = bytes(sample.payload)
        if len(payload) >= 16:
            ts_phone_ns, seq = struct.unpack_from("<qq", payload, 0)
            jpeg_bytes = payload[16:]
            synced_rt_ns = self.estimator.to_host_realtime_ns(ts_phone_ns, arrival_bt)
        else:
            jpeg_bytes = payload
            synced_rt_ns = realtime_ns()

        if self.on_camera:
            self.on_camera(jpeg_bytes, synced_rt_ns)

    def close(self):
        self._udp_running = False
        try:
            self._udp_sock.close()
        except Exception:
            pass
        self.session.close()

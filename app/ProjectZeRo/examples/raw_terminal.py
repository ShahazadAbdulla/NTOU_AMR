#!/usr/bin/env python3
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque

from project_zero_sdk.protocol import GpsSample, ImuSample
from project_zero_sdk.transport import ZenohSensorNode


def get_local_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"


def get_connected_peers(port: int = 7447) -> set[str]:
    """Checks OS socket table for established TCP connections on port 7447."""
    peers = set()
    try:
        out = subprocess.check_output(["ss", "-tn", f"sport = :{port}"], text=True)
        for line in out.splitlines()[1:]:
            parts = line.split()
            if len(parts) >= 5 and parts[0] == "ESTAB":
                peer_ip = parts[4].rsplit(":", 1)[0].replace("[", "").replace("]", "")
                if peer_ip not in ("127.0.0.1", "::1"):
                    peers.add(peer_ip)
    except Exception:
        pass
    return peers


class EdgeLogger:
    def __init__(self):
        self.lock = threading.Lock()
        self.imu_streaming = False
        self.current_imu_mode = None
        self.gps_streaming = False
        self.cam_streaming = False
        self.last_imu_ts = 0.0
        self.last_gps_ts = 0.0
        self.last_cam_ts = 0.0
        self.imu_window = deque(maxlen=400)
        self.gps_window = deque(maxlen=30)
        self.cam_window = deque(maxlen=60)
        self.latest_imu: ImuSample | None = None
        self.latest_gps: GpsSample | None = None
        self.latest_cam_bytes = 0

    @staticmethod
    def log_event(tag: str, color_code: str, msg: str):
        ts = time.strftime("%H:%M:%S")
        # Clear the single-line status bar, print the event once, and flush
        sys.stdout.write(f"\r\033[K[{ts}] \033[{color_code}m[{tag}]\033[0m {msg}\n")
        sys.stdout.flush()

    def on_imu(self, sample: ImuSample, synced_ns: int):
        now = time.monotonic()
        mode_str = "UNIVERSAL (Quat+Gyro+Accel)" if sample.mode == 2 else ("FUSED" if sample.mode == 0 else "RAW")
        with self.lock:
            self.last_imu_ts = now
            self.imu_window.append(now)
            self.latest_imu = sample
            if not self.imu_streaming:
                self.imu_streaming = True
                self.current_imu_mode = sample.mode
                self.log_event("STREAM", "1;32", f"IMU data stream ACTIVE — Mode: {mode_str}")
            elif self.current_imu_mode != sample.mode:
                self.current_imu_mode = sample.mode
                self.log_event("STREAM", "1;36", f"IMU mode switched to {mode_str}")

    def on_gps(self, sample: GpsSample, synced_ns: int):
        now = time.monotonic()
        with self.lock:
            self.last_gps_ts = now
            self.gps_window.append(now)
            self.latest_gps = sample
            if not self.gps_streaming:
                self.gps_streaming = True
                self.log_event(
                    "STREAM",
                    "1;32",
                    f"GPS telemetry stream ACTIVE — Lock: ({sample.latitude:.5f}, {sample.longitude:.5f})",
                )

    def on_camera(self, jpeg_bytes: bytes, synced_ns: int):
        now = time.monotonic()
        with self.lock:
            self.last_cam_ts = now
            self.cam_window.append(now)
            self.latest_cam_bytes = len(jpeg_bytes)
            if not self.cam_streaming:
                self.cam_streaming = True
                kb = len(jpeg_bytes) / 1024.0
                self.log_event("STREAM", "1;32", f"Camera frame stream ACTIVE — Frame: {kb:.1f} KB")

    def tick_status(self, connected_peers: set[str]):
        now = time.monotonic()
        with self.lock:
            # Detect stream timeout (no IMU packets for > 1.5s)
            if self.imu_streaming and (now - self.last_imu_ts) > 1.5:
                self.imu_streaming = False
                self.log_event("STREAM", "1;33", "IMU data stream HALTED (no packets for 1.5s)")

            if self.gps_streaming and (now - self.last_gps_ts) > 3.0:
                self.gps_streaming = False
                self.log_event("STREAM", "1;33", "GPS telemetry stream HALTED")

            if self.cam_streaming and (now - self.last_cam_ts) > 2.0:
                self.cam_streaming = False
                self.log_event("STREAM", "1;33", "Camera frame stream HALTED")

            imu_hz = len([t for t in self.imu_window if now - t <= 1.0])
            gps_hz = len([t for t in self.gps_window if now - t <= 2.0]) / 2.0
            cam_fps = len([t for t in self.cam_window if now - t <= 1.0])
            cam_kb = self.latest_cam_bytes / 1024.0
            imu = self.latest_imu

        # Update single bottom status line in place (no scrolling spam)
        is_active = self.imu_streaming or self.cam_streaming or self.gps_streaming
        if not connected_peers and not is_active:
            status = "\033[33m● WAITING FOR PHONE CONNECTION...\033[0m"
        elif connected_peers and not is_active:
            peers_str = ", ".join(connected_peers)
            status = f"\033[36m● PHONE CONNECTED ({peers_str}) — Waiting for 'START STREAMING'\033[0m"
        else:
            val_str = ""
            if imu:
                val_str = f"qW={imu.qw:+.2f} qZ={imu.qz:+.2f} gz={imu.gz:+.3f}"
            cam_str = f" | CAM: {cam_fps:2d} FPS ({cam_kb:.0f} KB)" if self.cam_streaming else ""
            status = f"\033[32m● STREAMING\033[0m | IMU: {imu_hz:3d} Hz ({val_str}){cam_str} | GPS: {gps_hz:.1f} Hz"

        sys.stdout.write(f"\r\033[K  {status}")
        sys.stdout.flush()


def main():
    host_ip = get_local_ip()
    logger = EdgeLogger()

    print("\033[1;36m============================================================\033[0m")
    print("\033[1;36m  PROJECT ZERO — RAW ZENOH DIAGNOSTIC MONITOR\033[0m")
    print("\033[1;36m============================================================\033[0m")
    print(f"  Host Wi-Fi IP : \033[1;32m{host_ip}\033[0m")
    print("  Listen Port   : \033[1;37m7447 (TCP & UDP Unicast)\033[0m")
    print("  Zenoh Mode    : \033[1;37mROUTER (Accepts Client & Peer connections)\033[0m")
    print("\033[1;36m------------------------------------------------------------\033[0m")

    node = ZenohSensorNode(
        mode="router",
        endpoint="tcp/0.0.0.0:7447",
        on_imu=logger.on_imu,
        on_gps=logger.on_gps,
        on_camera=logger.on_camera,
    )
    logger.log_event("SERVER", "1;34", f"Zenoh Router listening on {host_ip}:7447")

    def shutdown_handler(sig, frame):
        sys.stdout.write("\r\033[K")
        logger.log_event("SERVER", "1;31", "Shutting down Zenoh session and releasing port 7447...")
        try:
            node.close()
        except Exception:
            pass
        os._exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    active_peers: set[str] = set()
    while True:
        current_peers = get_connected_peers(7447)
        for p in current_peers - active_peers:
            logger.log_event("NETWORK", "1;32", f"Phone CONNECTED from {p}:7447")
        for p in active_peers - current_peers:
            logger.log_event("NETWORK", "1;31", f"Phone DISCONNECTED ({p})")
        active_peers = current_peers

        logger.tick_status(active_peers)
        time.sleep(0.2)


if __name__ == "__main__":
    main()

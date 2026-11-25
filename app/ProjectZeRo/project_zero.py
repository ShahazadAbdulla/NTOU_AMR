#!/usr/bin/env python3
import os
import signal
import socket
import subprocess
import threading
import time
from collections import deque

import rclpy
from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.prompt import Prompt
from rich.table import Table
from rich.text import Text

from project_zero_sdk.protocol import GpsSample, ImuSample
from project_zero_sdk.ros2_adapter import ProjectZeroRos2Publisher
from project_zero_sdk.time_sync import ClockSyncEstimator
from project_zero_sdk.transport import ZenohSensorNode

console = Console()

BANNER = r"""
 ____  ____   ___      _ _____ ____ _____   _____ _____ ____   ___  
|  _ \|  _ \ / _ \    | | ____/ ___|_   _| |__  /| ____|  _ \ / _ \ 
| |_) | |_) | | | |_  | |  _|| |     | |     / / |  _| | |_) | | | |
|  __/|  _ <| |_| | |_| | |__| |___  | |    / /_ | |___|  _ <| |_| |
|_|   |_| \_\\___/ \___/|_____\____| |_|   /____||_____|_| \_\\___/ 
"""


def get_local_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"


def get_connected_peers(port: int = 7447) -> set[str]:
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


class TelemetryState:
    def __init__(self):
        self.lock = threading.Lock()
        self.imu_times = deque(maxlen=450)
        self.gps_times = deque(maxlen=30)
        self.cam_times = deque(maxlen=60)
        self.last_packet_mono = 0.0
        self.last_imu: ImuSample | None = None
        self.last_gps: GpsSample | None = None
        self.last_cam_kb: float = 0.0
        self.events = deque(maxlen=6)

    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        with self.lock:
            self.events.append(f"[{ts}] {msg}")

    def record_imu(self, sample: ImuSample):
        now = time.monotonic()
        with self.lock:
            self.imu_times.append(now)
            self.last_packet_mono = now
            self.last_imu = sample

    def record_gps(self, sample: GpsSample):
        now = time.monotonic()
        with self.lock:
            self.gps_times.append(now)
            self.last_packet_mono = now
            self.last_gps = sample

    def record_cam(self, size_bytes: int):
        now = time.monotonic()
        with self.lock:
            self.cam_times.append(now)
            self.last_packet_mono = now
            self.last_cam_kb = size_bytes / 1024.0

    def get_metrics(self):
        now = time.monotonic()
        with self.lock:
            streaming = (now - self.last_packet_mono) < 1.5 and self.last_packet_mono > 0
            imu_hz = len([t for t in self.imu_times if now - t <= 1.0])
            gps_hz = len([t for t in self.gps_times if now - t <= 2.0]) / 2.0
            cam_fps = len([t for t in self.cam_times if now - t <= 1.0])
            return streaming, imu_hz, gps_hz, cam_fps, self.last_imu, self.last_gps, self.last_cam_kb, list(self.events)


def build_dashboard(
    host_ip: str,
    mode: str,
    endpoint: str,
    peers: set[str],
    state: TelemetryState,
    estimator: ClockSyncEstimator,
) -> Layout:
    streaming, imu_hz, gps_hz, cam_fps, last_imu, last_gps, cam_kb, events = state.get_metrics()

    layout = Layout()
    layout.split_column(
        Layout(name="header", size=9),
        Layout(name="body", ratio=1),
        Layout(name="logs", size=9),
    )
    layout["body"].split_row(
        Layout(name="network"),
        Layout(name="sensors"),
    )

    if streaming:
        status_badge = Text("● STREAMING ACTIVE", style="bold green")
    elif peers:
        status_badge = Text(f"● PHONE CONNECTED ({', '.join(peers)}) — READY", style="bold cyan")
    else:
        status_badge = Text("○ WAITING FOR PHONE CONNECTION", style="bold yellow")

    header_text = Text(BANNER.strip("\n"), style="bold cyan")
    header_text.append("\n  ROS 2 Edge Telemetry & Time-Sync Ground Station   |   Status: ")
    header_text.append_text(status_badge)
    layout["header"].update(Panel(header_text, border_style="cyan"))

    net_table = Table(show_header=False, box=None, padding=(0, 1))
    net_table.add_column("Key", style="bold white")
    net_table.add_column("Value", style="cyan")
    net_table.add_row("Host Wi-Fi IP", f"[bold green]{host_ip}[/bold green]  (Enter in Phone App)")
    net_table.add_row("Zenoh Port", "7447 (TCP / UDP Unicast)")
    net_table.add_row("Zenoh Mode", mode.upper())
    net_table.add_row("Connected Peers", ", ".join(peers) if peers else "None")
    net_table.add_row("", "")

    sync_label = (
        "[bold green]SYNCED (2-Way Burst)[/bold green]"
        if estimator.synced
        else "[yellow]UNSYNCED_FALLBACK (1-Way Min)[/yellow]"
    )
    net_table.add_row("Clock Sync", sync_label)
    net_table.add_row("Skew (ppm)", f"{estimator.slope * 1e6:.2f} ppm")
    net_table.add_row("Bursts OK / Fail", f"{estimator.bursts_ok} / {estimator.bursts_failed}")
    layout["network"].update(Panel(net_table, title="[bold]Network & Clock Sync[/bold]", border_style="blue"))

    sen_table = Table(show_header=True, header_style="bold magenta", box=None)
    sen_table.add_column("ROS 2 Topic")
    sen_table.add_column("Rate")
    sen_table.add_column("Live Payload")

    if last_imu:
        imu_val = (
            f"Q:[{last_imu.qx:+.2f},{last_imu.qy:+.2f},{last_imu.qz:+.2f},{last_imu.qw:+.2f}] "
            f"Gz:{last_imu.gz:+.3f}"
        )
    else:
        imu_val = "Waiting..."

    gps_val = (
        f"{last_gps.latitude:.5f}, {last_gps.longitude:.5f} | Hdg: {last_gps.bearing:.1f}°"
        if last_gps
        else "Waiting..."
    )
    cam_val = f"1280x720 JPEG ({cam_kb:.1f} KB/frame)" if cam_kb > 0 else "Waiting..."

    sen_table.add_row("/project_zero/imu/data", f"[bold green]{imu_hz:3d} Hz[/bold green]", imu_val)
    sen_table.add_row("/project_zero/camera/image/compressed", f"[bold green]{cam_fps:3d} FPS[/bold green]", cam_val)
    sen_table.add_row("/project_zero/gps/fix", f"[bold green]{gps_hz:3.1f} Hz[/bold green]", gps_val)
    sen_table.add_row("/project_zero/gps/bearing", f"[bold green]{gps_hz:3.1f} Hz[/bold green]", "Float64 Heading")
    sen_table.add_row("/project_zero/clock/status", "1.0 Hz", "DiagnosticStatus")
    layout["sensors"].update(Panel(sen_table, title="[bold]ROS 2 Output Streams[/bold]", border_style="green"))

    log_lines = "\n".join(events) if events else "Ready. Connect from Android app to start."
    layout["logs"].update(Panel(Text(log_lines, style="white"), title="[bold]System Events (Ctrl+C to Exit)[/bold]", border_style="white"))

    return layout


def main():
    console.clear()
    console.print(BANNER, style="bold cyan")

    host_ip = get_local_ip()
    console.print(f"[bold]Detected Active Host IP:[/bold] [bold green]{host_ip}[/bold green]\n")

    mode = Prompt.ask("Select Zenoh Session Mode", choices=["router", "peer", "client"], default="peer")
    endpoint = "tcp/127.0.0.1:7447" if mode == "client" else "tcp/0.0.0.0:7447"

    state = TelemetryState()
    estimator = ClockSyncEstimator()

    rclpy.init()
    ros_pub = ProjectZeroRos2Publisher(
        imu_topic="/project_zero/imu/data",
        gps_fix_topic="/project_zero/gps/fix",
        gps_bearing_topic="/project_zero/gps/bearing",
        camera_topic="/project_zero/camera/image/compressed",
        clock_status_topic="/project_zero/clock/status",
        estimator=estimator,
    )

    def on_imu(sample: ImuSample, stamp_ns: int):
        if state.last_imu is None:
            state.log("IMU Universal stream ACTIVE (408Hz Quat + Gyro + Accel)")
        state.record_imu(sample)
        ros_pub.publish_imu(sample, stamp_ns)

    def on_gps(sample: GpsSample, stamp_ns: int):
        if state.last_gps is None:
            state.log(f"GPS telemetry ACTIVE ({sample.latitude:.5f}, {sample.longitude:.5f})")
        state.record_gps(sample)
        ros_pub.publish_gps(sample, stamp_ns)

    def on_camera(jpeg_bytes: bytes, stamp_ns: int):
        if state.last_cam_kb == 0.0:
            state.log(f"Camera 720p stream ACTIVE ({len(jpeg_bytes)/1024.0:.1f} KB)")
        state.record_cam(len(jpeg_bytes))
        ros_pub.publish_camera(jpeg_bytes, stamp_ns)

    zenoh_node = ZenohSensorNode(
        mode=mode,
        endpoint=endpoint,
        on_imu=on_imu,
        on_gps=on_gps,
        on_camera=on_camera,
        estimator=estimator,
    )
    state.log(f"Zenoh [{mode.upper()}] listening on {host_ip}:7447")

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_pub,), daemon=True)
    spin_thread.start()

    def shutdown_handler(sig, frame):
        try:
            zenoh_node.close()
            ros_pub.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        os._exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    active_peers: set[str] = set()
    with Live(build_dashboard(host_ip, mode, endpoint, active_peers, state, estimator), refresh_per_second=8, screen=True) as live:
        while True:
            current_peers = get_connected_peers(7447)
            for p in current_peers - active_peers:
                state.log(f"Phone CONNECTED from {p}:7447")
            for p in active_peers - current_peers:
                state.log(f"Phone DISCONNECTED ({p})")
            active_peers = current_peers

            live.update(build_dashboard(host_ip, mode, endpoint, active_peers, state, estimator))
            time.sleep(0.15)


if __name__ == "__main__":
    main()

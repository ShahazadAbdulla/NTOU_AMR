import math
import threading
import time
from collections import deque


def boottime_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


def realtime_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_REALTIME)


class ClockSyncEstimator:
    """
    Maps Phone CLOCK_BOOTTIME -> Host CLOCK_BOOTTIME -> Host CLOCK_REALTIME.
    Automatically falls back to one-way rolling minimum offset when unsynced.
    """

    def __init__(self, skew_window_s: float = 600.0, max_slew_ppm: float = 500.0, future_clamp_ns: int = 5_000_000):
        self.skew_window_ns = int(skew_window_s * 1e9)
        self.max_slew_rate = max_slew_ppm * 1e-6
        self.future_clamp_ns = future_clamp_ns
        self.lock = threading.Lock()

        self.boot_id = None
        self.synced = False
        self.degraded = False
        self.theta_0 = 0.0
        self.t_ref_phone = 0
        self.slope = 0.0
        self.slew_target_offset = 0.0
        self.last_predict_phone_ns = None

        self.history = deque()
        self.one_way_window = deque(maxlen=200)

        self.bursts_ok = 0
        self.bursts_failed = 0
        self.consecutive_rejects = 0
        self.last_burst_host_ns = 0
        self.delta_best_ns = None
        self.last_delta_min_ns = 0
        self.last_delta_p99_ns = 0
        self.offset_std_ns = 0.0
        self.future_clamped_count = 0
        self.last_emitted_rt_ns = 0

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
            self.last_emitted_rt_ns = 0

    def update_one_way(self, ts_phone_ns: int, arrival_host_boot_ns: int):
        with self.lock:
            self.one_way_window.append(arrival_host_boot_ns - ts_phone_ns)

    def ingest_phone_burst_report(self, t_phone_ns: int, theta_hat: float, delta_min: float, boot_id: int) -> bool:
        if self.boot_id is not None and boot_id != self.boot_id:
            self.reset(new_boot_id=boot_id)
        elif self.boot_id is None:
            self.boot_id = boot_id

        with self.lock:
            self.last_delta_min_ns = int(delta_min)
            now_host = boottime_ns()
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

            if self.delta_best_ns is None or delta_min < self.delta_best_ns:
                self.delta_best_ns = delta_min

            theta_pred = self.theta_0 + self.slope * (t_phone_ns - self.t_ref_phone) + self.slew_target_offset
            residual = theta_hat - theta_pred
            if abs(residual) >= 100_000.0:
                self.slew_target_offset += residual

            self.history.append((t_phone_ns, float(theta_hat), float(delta_min)))
            cutoff = t_phone_ns - self.skew_window_ns
            while len(self.history) > 2 and self.history[0][0] < cutoff:
                self.history.popleft()
            self._fit_skew_locked()
            self.bursts_ok += 1
            self.degraded = False
            self.last_burst_host_ns = now_host
            return True

    def _fit_skew_locked(self):
        n = len(self.history)
        if n < 3:
            return
        t0 = self.history[0][0]
        xs = [(h[0] - t0) * 1e-9 for h in self.history]
        ys = [h[1] for h in self.history]
        mean_x, mean_y = sum(xs) / n, sum(ys) / n
        denom = sum((x - mean_x) ** 2 for x in xs)
        if denom > 1.0:
            slope_ns_per_s = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom
            self.slope = max(-200e-6, min(200e-6, slope_ns_per_s * 1e-9))
            residuals = [y - (mean_y + slope_ns_per_s * (x - mean_x)) for x, y in zip(xs, ys)]
            self.offset_std_ns = math.sqrt(sum(r * r for r in residuals) / n)

    def to_host_realtime_ns(self, ts_phone_ns: int, arrival_host_boot_ns: int, test_bias_ns: int = 0) -> int:
        with self.lock:
            if not self.synced:
                self.one_way_window.append(arrival_host_boot_ns - ts_phone_ns)
                stamp_boot_ns = int(ts_phone_ns + min(self.one_way_window))
            else:
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
                stamp_boot_ns = int(ts_phone_ns + round(theta))

            rt_now = realtime_ns()
            bt_now = boottime_ns()
            k = rt_now - bt_now
            stamp_rt_ns = stamp_boot_ns + k + test_bias_ns
            if stamp_rt_ns > rt_now + self.future_clamp_ns:
                stamp_rt_ns = rt_now
                self.future_clamped_count += 1
            if stamp_rt_ns <= self.last_emitted_rt_ns:
                stamp_rt_ns = self.last_emitted_rt_ns + 1_000
            self.last_emitted_rt_ns = stamp_rt_ns
            return stamp_rt_ns

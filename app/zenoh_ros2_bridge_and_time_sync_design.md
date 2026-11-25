# Zenoh → ROS 2 Bridge and Phone Clock Synchronization — Design Specification

**Project:** NTOU / Project ZeRo — Phase 1 (phone IMU/GPS/camera into a real robot)
**Host:** NVIDIA Jetson Orin Nano, ROS 2 Jazzy, middleware `rmw_zenoh`
**Phone:** Android (Samsung Galaxy S23), `org.eclipse.zenoh:zenoh-kotlin-android:1.10.1`
**Companion document:** `project_zero_architecture_engineering_manual.md`
**Status:** design proposal, derived from the current repository state. Numbers marked *(measure)* are estimates that must be confirmed on the real robot.

---

## 1. Purpose and scope

This document specifies two coupled things:

1. **A Zenoh → ROS 2 bridge** (`phone_zenoh_bridge`) running on the Jetson, which receives the phone's raw Zenoh samples and republishes them as standard ROS 2 messages on `/imu/data`, `/gps/fix` and (later) `/camera/image_raw`.
2. **A clock synchronization subsystem** inside that same bridge, which maps the phone's own monotonic time onto ROS time, so that `header.stamp` on every published message is meaningful to `robot_localization`, `tf2`, `message_filters` and `ros2_control`.

Scope excludes: VIO (phase 3), simulation, dora-rs (dropped), and any change to the robot's existing controller stack.

---

## 2. Verified current state (what the code actually does today)

Read from this checkout, not assumed:

| Fact | Location |
|---|---|
| IMU payload is a 33-byte little-endian struct `<qB6f` = `int64 ts_ns`, `uint8 mode`, `6 × float32` | `app/ZenohTelemetry/.../sensors/TelemetryManager.kt:36-37`, `app/ProjectZeRo/ingestion_node.py:39` |
| GPS payload is a 48-byte struct `<q3d4f` | `TelemetryManager.kt:40-41`, `ingestion_node.py:68` |
| App-level key expressions are `phone/sensors/imu`, `phone/sensors/camera`, `phone/sensors/gps` | `transport/ZenohTransport.kt:42,47,52` |
| IMU is `BEST_EFFORT` + `CongestionControl.DROP` + `express=true`; camera and GPS are `RELIABLE` | `ZenohTransport.kt:43-54` |
| Multicast scouting is disabled; pure unicast to one endpoint | `ZenohTransport.kt:29-36` |
| **The timestamp is `System.nanoTime()` taken inside `onSensorChanged`, not `SensorEvent.timestamp`** | `TelemetryManager.kt:88`, and `:136` for GPS |
| In raw mode, the packet is emitted when the **gyro** event arrives and pairs the **most recent accel** sample | `TelemetryManager.kt:92-109` |
| The host side currently unpacks to JSON and pushes into dora | `ingestion_node.py:34-62` |
| No ROS 2 publisher exists in this checkout; the bridge is described but not implemented | — |

Two consequences follow immediately and they are not optional fixes:

- `System.nanoTime()` measures **when the app was scheduled**, not **when the sensor sampled**. The delay is 1–5 ms typically and can reach tens or hundreds of milliseconds when the phone buffers events.
- `System.nanoTime()` is `CLOCK_MONOTONIC`, which **stops during deep sleep**, while the sensor timestamp is on a boot-based clock that **includes** sleep. Mixing them creates a permanent silent offset after the first suspend.

---

## 3. Decision summary

**Q: Is time synchronization necessary?**
Yes. The EKF integrates the IMU and compares it against wheel odometry, LiDAR and TF. All of those comparisons are made at `header.stamp`. An IMU stamped in the phone's own time base (nanoseconds since *the phone* booted) is not comparable to anything on the robot, and a constant offset alone will bias attitude and velocity.

**Q: Is the Zenoh queryable idea sound?**
Yes, with corrections. It is, in effect, **NTP implemented with application timestamps over the Zenoh query/reply path** — the only mechanism available on a non-rooted phone. Its accuracy floor is set by **network path asymmetry**, not by how often you query. So the protocol is right, but:

1. Fix the stamping point first (`SensorEvent.timestamp`, per channel). This is the largest single error term and costs no protocol work.
2. Re-query every **30 s** with event triggers, not every 10 s. At 20 ppm, drift over 10 s is 0.2 ms — below the network floor, so 10 s buys nothing but risk. Keep a 10 s *health probe* if you want early warning.
3. Do **not** send one query. Send a short burst and keep the lowest-round-trip sample; add a drift (skew) regression and a slew/step policy.
4. Add a one-way cross-check using the bridge's own arrival time. It is free and detects a bad offset that the round-trip test cannot see.

**Expected accuracy:** ~1 ms best case on a quiet 5 GHz link, **1–3 ms** realistic 1σ, with occasional 10–100 ms excursions caused by Wi-Fi power save. **Required accuracy:** total stamp error ≤ ~5 ms for the IMU, with ≤1 ms being comfortable.

---

## 4. Why stamps matter to each consumer

| Consumer | How it uses `header.stamp` | What a bad stamp does |
|---|---|---|
| `robot_localization` (EKF) | Transforms the measurement into the world/base frame at that exact time via `tf2`, and uses inter-arrival `dt` for the prediction step | Out-of-order or too-old data is not re-published (`permit_corrected_publication` defaults `false`); future-dated data is held until the filter catches up; a wrong `dt` distorts the process covariance |
| `tf2` | `lookupTransform` interpolates the buffer at the requested time (default `cache_time` 10 s) | A stamp beyond the buffer throws `ExtrapolationException`; a future stamp asks for a transform that does not exist yet |
| `message_filters` | `ApproximateTime` matches messages within `slop` | If phone and host clocks are offset by more than `slop`, nothing ever matches; `queue_size` must cover the rate mismatch |
| `ros2_control` | Controller manager runs the read/update/write loop on a steady clock, but passes **ROS time** into controller `update()`; broadcasters stamp output with that time | A stamp that jumps backwards stalls downstream consumers; stamps inconsistent with the loop time make RViz and bag playback lie |
| Bags / replay / debugging | Time is the primary index | Unsynced stamps make the recording uninterpretable |

Reference behaviour in Jazzy: `ros2_control` controller-manager documentation (clock section), `robot_localization` `state_estimation_nodes` docs, `geometry2` `tf2_ros::Buffer` API, `message_filters` `ApproximateTime` policy.

---

## 5. Required accuracy, derived

Definitions: `Δt` = error between the stamped time and the true sampling instant.

```
attitude error      Δθ = ω · Δt                     [rad]
specific-force leak Δa ≈ g · ω · Δt                 [m/s²]   (gravity leaking into the horizontal channel)
velocity error      Δv = a · Δt                     [m/s]
```

At the phone's fused rate of 204 Hz the sample period is **4.9 ms**; at 408 Hz it is **2.45 ms**.

| `Δt` | Δθ at ω = 1 rad/s | Δv at a = 2 m/s² | Verdict |
|---|---|---|---|
| 0.5 ms | 0.03° | 1 mm/s | negligible |
| 1 ms | 0.06° | 2 mm/s | comfortable |
| 2.5 ms | 0.14° | 5 mm/s | good — half a sample period |
| 5 ms | 0.29° | 10 mm/s | acceptable ceiling |
| 10 ms | 0.57° | 20 mm/s | visible drift, and ~2 sample periods of smear |
| 50 ms | 2.9° | 100 mm/s | wrong; motion is attributed to the wrong instant |

**Budget target:** total stamp error **≤ 5 ms p99**, **≤ 1 ms typical**.

Note the pattern: a *constant* offset of a few milliseconds is survivable; a *variable* offset is not, because it corrupts the time axis rather than shifting it.

---

## 6. Phone clock domains — and the bug in the current code

| API | Underlying clock | Includes deep sleep? | Notes |
|---|---|---|---|
| `System.nanoTime()` | `CLOCK_MONOTONIC` | **No** — freezes in suspend | Cheap, but wrong base for sensor data |
| `SystemClock.elapsedRealtimeNanos()` | `CLOCK_BOOTTIME` | Yes | Same base as the sensor timestamps |
| `SensorEvent.timestamp` | Boot-based, specified to be synchronized with `elapsedRealtimeNanos()` | Yes | **The hardware sampling instant.** This is the value to use |
| `Location.getElapsedRealtimeNanos()` | `CLOCK_BOOTTIME` | Yes | Use this for GPS instead of `System.nanoTime()` |

Why this is the most important fix in the whole design:

- `SensorEvent.timestamp` is created by the sensor hub at the moment the physical conversion finished. It is immune to app scheduling, GC pauses, CPU frequency scaling and thermal throttling.
- `System.nanoTime()` at callback time adds the whole delivery chain: binder, looper queue, app code, and — if the phone buffered events — the entire buffering window. It also destroys intra-batch spacing: when 40 buffered samples are delivered back-to-back, 40 different physical instants receive nearly the same stamp.
- If the app is ever suspended, the two clocks diverge permanently and nothing downstream can detect it.

Android's own documentation states `SensorEvent.timestamp` uses the same time base as `SystemClock.elapsedRealtimeNanos()`. Verify on the specific S23 build *(measure)* — vendor HALs occasionally deviate.

### 6.1 Raw-mode pairing bias

Raw mode emits on the gyro callback using the most recent accelerometer values. So the two channels inside one packet are from **different instants**:

- at 408 Hz, accelerometer age is uniform in `[0, 2.45] ms`, mean ≈ 1.2 ms;
- if accel runs at 200 Hz while gyro runs at 400 Hz, the age is up to 5 ms.

At ω = 1 rad/s this misalignment leaks roughly `g · ω · 1.2 ms ≈ 12 mm/s²` of gravity into the horizontal channel — comparable to the accelerometer's own noise floor.

**Fix (required for phase 1):** timestamp each channel separately and carry both stamps (see the v2 struct in §10).

---

## 7. Architecture

```
   ┌──────────────────────── Samsung Galaxy S23 ────────────────────────┐
   │  SensorManager (sensor hub)                                        │
   │      │ SensorEvent.timestamp  (CLOCK_BOOTTIME, ns)                 │
   │      ▼                                                             │
   │  TelemetryManager  ──►  binary struct (v2: per-channel stamps)     │
   │      │                                                             │
   │      ▼                                                             │
   │  ZenohTransport (zenoh-kotlin 1.10.1, BEST_EFFORT/express, unicast)│
   │      │                                                             │
   │      │  ① publish  phone/sensors/imu      ──────────────┐          │
   │      │  ② get      phone/clock/query      ──────────┐   │          │
   │      └──────────────────────────────────────────────│───│──────────┘
                                                        │   │   Wi-Fi
   ┌───────────────────── Jetson Orin Nano ──────────────│───│──────────┐
   │                                                     ▼   ▼          │
   │  zenohd router (single hub, tcp/0.0.0.0:7447)                      │
   │        │                     ▲                                     │
   │        │                     │ reply: t2, t3, host clocks          │
   │        ▼                     │                                     │
   │  ┌──────────────────────────────────────────┐                      │
   │  │ phone_zenoh_bridge  (one process)        │                      │
   │  │  • plain Zenoh session (app keys)        │                      │
   │  │  • queryable  phone/clock/query          │                      │
   │  │  • estimator: offset + drift (ppm)       │                      │
   │  │  • ROS 2 node via rclpy/rclcpp           │                      │
   │  └──────────────────────────────────────────┘                      │
   │        │                                                           │
   │        ├─► /imu/data        sensor_msgs/Imu                        │
   │        ├─► /gps/fix         sensor_msgs/NavSatFix                  │
   │        ├─► /phone/clock/status  diagnostic_msgs/DiagnosticStatus   │
   │        └─► TF: base_link → phone_imu_link (static, from URDF)      │
   │                                                                    │
   │  robot_localization EKF ◄── /imu/data, /wheel/odom, /scan          │
   │  ros2_control           ◄── (stamps from its own loop clock)       │
   └────────────────────────────────────────────────────────────────────┘
```

### 7.1 Critical design decision — how the bridge talks to ROS 2

The robot runs `rmw_zenoh`. It is technically possible for a non-ROS Zenoh client to publish straight into the rmw key space, but this is **explicitly out of scope and not recommended**:

- `rmw_zenoh` topic key expressions in Jazzy are `<domain_id>/<fully_qualified_name>/<type_name>/<type_hash>`, e.g.
  `0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18`
- Payloads must be **CDR-serialized** and carry the rmw_zenoh **attachment** (sequence number, timestamp, publisher GID).
- The type hash (REP-2016 `RIHS01_...`) and the QoS-encoded liveliness tokens must also match, or discovery silently fails.

That is a version-locked reverse-engineering exercise with no error reporting when it breaks. **Therefore the bridge is a normal ROS 2 node** (rclpy or rclcpp) that additionally opens its own plain Zenoh session on the application keys. Two sessions in one process is fine and normal.

`zenoh-bridge-ros2dds` is **not** used: it bridges to CycloneDDS-style graphs with its own key scheme and cannot interoperate with `rmw_zenoh`.

---

## 8. Zenoh key space

Two key spaces coexist and must not be confused:

| Space | Example | Who owns it |
|---|---|---|
| **Application keys** (phone ↔ bridge) | `phone/sensors/imu`, `phone/sensors/gps`, `phone/sensors/camera`, `phone/clock/query` | This design |
| **rmw_zenoh keys** (bridge ↔ ROS nodes) | `0/imu/data/sensor_msgs::msg::dds_::Imu_/RIHS01_...` | Generated automatically by `rmw_zenoh_cpp` |

Rules:

1. Never hand-craft a key inside the `0/...` or `@ros2_lv/...` space.
2. Keep the application keys under a single prefix so they are easy to filter, count and secure: `phone/**`.
3. Because the same Zenoh graph carries both, ensure the bridge's app-level session subscribes with explicit key selectors (`phone/sensors/*`), not `**`.
4. Reserve `phone/clock/query` for the queryable. Do not reuse it as a publisher key.

---

## 9. Message schema

### 9.1 Existing v1 (keep for compatibility)

| Stream | Key | Size | Format |
|---|---|---|---|
| IMU | `phone/sensors/imu` | 33 B | `<qB6f` → `ts_ns:int64`, `mode:uint8`, 6 × `float32` |
| GPS | `phone/sensors/gps` | 48 B | `<q3d4f` → `ts_ns:int64`, `lat,lon,alt:float64`, `speed,acc,bearing,bearing_acc:float32` |

`mode = 0` → `[qx, qy, qz, qw, 0, 0]` (fused game rotation vector)
`mode = 1` → `[ax, ay, az, gx, gy, gz]` (linear acceleration without gravity, then gyroscope rad/s)

Note that `TYPE_LINEAR_ACCELERATION` already removes gravity, which is exactly what `sensor_msgs/Imu.linear_acceleration` expects. Do not apply a gravity-removal step in the bridge.

### 9.2 Proposed v2 IMU (recommended for phase 1)

42 bytes in Python `struct` (`<qqBB6f`):

| Field | Type | Meaning |
|---|---|---|
| `ts_gyro_ns` | int64 | gyro sample time, `CLOCK_BOOTTIME` ns |
| `ts_accel_ns` | int64 | accel sample time, `CLOCK_BOOTTIME` ns (0 if not applicable) |
| `mode` | uint8 | 0 = fused, 1 = raw |
| `flags` | uint8 | bit0 clock synced, bit1 gyro valid, bit2 accel valid |
| `v0..v5` | 6 × float32 | as in v1 |

Publish it on `phone/sensors/imu` and disambiguate by payload length (42 vs 33), or use `phone/sensors/imu/v2`. Length-based dispatch is simplest and lets the phone and bridge be upgraded independently.

**Packing caveat:** Python's `<` prefix means *no alignment*, so the payload is exactly 42 bytes. A C/C++ compiler would insert 2 bytes of padding after the two `uint8` fields and produce 44. If a C++ bridge ever reads this struct directly, it must use `#pragma pack(1)` / `__attribute__((packed))`, or the fields will be misread. Prefer explicit byte offsets over a cast.

### 9.3 Clock query/response payload

Query payload (`phone/clock/query`, 28 bytes, `<qqqi`):

| Field | Type | Meaning |
|---|---|---|
| `t1_ns` | int64 | phone `CLOCK_BOOTTIME` immediately **before** sending |
| `boot_id` | int64 | hash of a per-boot random value, so the host can detect a phone reboot |
| `seq` | int64 | monotonically increasing exchange counter |
| `want` | int32 | bitmask of requested fields (future-proofing) |

Reply payload (48 bytes, `<qqqqqq`):

| Field | Type | Meaning |
|---|---|---|
| `t1_ns` | int64 | echo of the client value (guards against stale replies) |
| `t2_ns` | int64 | host `CLOCK_BOOTTIME` immediately **after** receiving |
| `t3_ns` | int64 | host `CLOCK_BOOTTIME` immediately **before** replying |
| `t4_ns` | int64 | filled by the phone on arrival (phone BOOTTIME) |
| `host_realtime_ns` | int64 | host `CLOCK_REALTIME` taken with `t3` |
| `host_boot_ns` | int64 | host `CLOCK_BOOTTIME` taken with `t3` (redundant, for diagnostics) |

**Do not hardcode these sizes in validation logic.** Compute them with `struct.calcsize("<qqqi")` / `struct.calcsize("<qqqqqq")`, or compare against a declared constant in one place. A wrong literal turns every valid packet into a `struct.error` under load.

**Why BOOTTIME on both sides:** both clocks keep counting through suspend, so the offset between them stays valid across doze. If the phone used `CLOCK_MONOTONIC`, every suspend would silently break the mapping.

---

## 10. Time-sync subsystem

### 10.1 The exchange, and what it can and cannot measure

Let the phone clock be `C` and the host boot clock be `S`, with true offset `θ = S − C` and one-way delays `d1` (phone→host) and `d2` (host→phone):

```
t2 = t1 + θ + d1
t4 = t3 − θ + d2
```

Estimates:

```
θ̂ = ½ [ (t2 − t1) + (t3 − t4) ]      offset estimate
δ  =     (t4 − t1) − (t3 − t2)       round-trip delay, excluding server processing
```

The error is dominated by exactly one thing:

```
θ̂ − θ = ½ (d1 − d2)        →        |θ̂ − θ| ≤ δ/2
```

The residual is **half the path asymmetry**. (Timestamp-read jitter on the host and oscillator drift during the exchange add small extra terms, but they are microsecond-scale here.) More queries do not reduce the asymmetry term. Only statistics do: Wi-Fi queueing delay is non-negative, so the *minimum-δ* sample is the one with the least queueing and therefore the least asymmetry. That is the whole justification for a burst-and-filter design, and it is the same argument NTP uses (RFC 5905 clock filter).

Implementation detail that silently ruins accuracy: capture `t2` **immediately after** the query arrives and `t3` **immediately before** sending the reply. Any work in between (JSON encoding, logging, GIL contention) is counted as `d2` and biases `θ̂` by `−processing/2`.

### 10.2 Estimator

**Time base convention used throughout this section and §11:** `θ` maps **phone `CLOCK_BOOTTIME` → host `CLOCK_BOOTTIME`**. Therefore every `t` in the estimator is a *phone* BOOTTIME value, `t₀` is a phone BOOTTIME reference (use `t₀ = 0`, i.e. phone boot), and the regression slope `s` is dimensionless. Never feed host timestamps into this fit.

Per burst (N = 8 exchanges, 20–50 ms apart):

1. Discard non-monotonic or stale-reply samples (`t1` echo mismatch, `δ < 0`).
2. **Min-delay selection:** keep the sample with the smallest `δ`; tie-break by the median of the rest.
3. **Marzullo / median agreement:** intersect the intervals `[θ̂ᵢ − δᵢ/2, θ̂ᵢ + δᵢ/2]`; if the surviving set is not a majority, treat the burst as failed.
4. Publish a per-burst estimate `θ̂` and a quality value `q = δ_min` (smaller is better).

Across bursts (window ≥ 10 minutes):

5. **Skew regression:** fit `θ(t) = θ₀ + s·(t − t₀)` by least squares over accepted bursts. `s` is the fractional frequency error; report `s·1e6` in **ppm**. This is what actually removes drift. Re-fitting is why re-sync intervals can be long.
6. **Prediction:** at publish time use `θ(t) = θ₀ + s·(t − t₀)`; no query is needed per sample.
7. **Outlier gate:** reject a burst if `δ_min > δ_best + max(1 ms, 0.5·δ_best, 3σ_δ)` or `|θ̂ − θ_pred| > 5σ_θ`. Require 3 consecutive rejects before believing a real change.

### 10.3 Cost of drift — why 10 s is over-engineered

Accumulated offset = `ppm × Δt`:

| Oscillator | 10 s | 30 s | 60 s | 1 h |
|---|---|---|---|---|
| 10 ppm | 0.10 ms | 0.30 ms | 0.60 ms | 36 ms |
| 20 ppm | 0.20 ms | 0.60 ms | 1.20 ms | 72 ms |
| 50 ppm | 0.50 ms | 1.50 ms | 3.00 ms | 180 ms |

With skew regression active, the residual is estimation noise plus the regression error, not raw drift. **Recommendation: a full re-sync every 30 s, plus event-triggered syncs** (session reconnect, Wi-Fi roam, phone wake from doze, `boot_id` change, host `NTP` step detected). Keep a 10 s lightweight probe if you want an early warning signal, but do not let it cause a stamp jump.

### 10.4 Step vs slew

| Condition | Action |
|---|---|
| `|Δθ| < 100 µs` | ignore |
| `100 µs … 1 ms` | slew: move the map by at most 500 ppm (0.5 ms per second) so `dt` never jumps |
| `> 1 ms`, confirmed by 3 bursts, **or** before the first publish | step, and emit a `clock_step` diagnostic with the size |
| Phone reboot (`boot_id` change) or unsynced start | hard re-init: flush buffers, re-sync, then resume |

Rule: **never rewrite an already-published stamp.** Change the affine map, and prefer slewing once data is flowing. A 1 ms residual is cheaper than a discontinuity in the EKF's time axis.

**Exception — host clock:** the `k` term in §10.5 is re-sampled per message, so a host NTP step appears immediately in the output as a jump. That is not avoidable while publishing ROS time, and it is not the same failure mode as a phone-offset jump: the robot's own consumers saw the host clock jump too. Detect it, log one `clock_step` event with the magnitude, and do not attempt to mask it by slewing.

### 10.5 BOOTTIME → ROS time conversion

Keep the map in boot-clock terms, convert only at publish:

```
stamp_host_boot = phone_ts_boot_ns + θ(t)                       [ns, host BOOTTIME]
k               = host_realtime_ns − host_boot_ns               [ns], sampled at publish time
stamp_ros_ns    = stamp_host_boot + k
```

Because `k` is re-sampled for every message, an NTP step or manual clock change on the Jetson is picked up automatically **without** needing to re-sync with the phone. Detect a `k` jump and log one `clock_step` event; do not silently absorb it.

---

## 11. Bridge node — algorithm

### 11.1 Startup sequence

```
open ROS 2 node (parameter use_sim_time must be false; warn and refuse if true)
open Zenoh session (client or peer, unicast to configured endpoint)
declare queryable "phone/clock/query"          # before opening publishers
                                                                
wait_until_synced(timeout = 5 s, retries with backoff):
    for burst in 1..3:
        for i in 1..8:
            t1 = boottime_ns()
            replies = get("phone/clock/query", payload{ t1, boot_id, seq }, timeout = 300 ms)
            t4 = boottime_ns()
            keep sample(θ̂, δ)
            sleep(20..50 ms jittered)
        if accept(burst): break

declare subscribers phone/sensors/imu, /gps, /camera
declare ROS publishers /imu/data, /gps/fix, /phone/clock/status
declare timer(10 s): health probe  (report only)
declare timer(30 s): resync burst  (may step/slew)
```

If no valid offset is obtained: **still publish**, but set `flags.clock_synced = 0`, use `host_arrival_time − measured_one_way_estimate` as a best-effort stamp, and publish a `WARN` diagnostic. Never fabricate a synchronized stamp.

### 11.2 Queryable handler

```
on_query(query):
    t2 = boottime_ns()                     # immediately on entry, before decoding
    try:
        (t1, boot_id, seq) = decode(query.payload)   # raises on length/magic mismatch
    except DecodeError:
        count("bad_query_payload")
        return                              # do not reply, do not crash the handler
    t3 = boottime_ns()
    reply = { t1_echo, t2, t3, 0, realtime_ns(), boottime_ns() }
    query.reply("phone/clock/query", encode(reply))    # immediately, no logging first
```

Keep this handler allocation-free and free of ROS calls. Do not log a line per exchange at debug level either — logging inside the handler is counted as server processing. A malformed or hostile payload must never propagate an exception into the Zenoh runtime thread.

### 11.3 IMU ingest → `sensor_msgs/Imu`

```
on_imu_sample(payload, arrival_ns):
    n = len(payload)
    if   n == IMU_V2_SIZE: s = IMU_V2.unpack(payload); ts_phone = s.ts_gyro_ns
    elif n == IMU_V1_SIZE: s = IMU_V1.unpack(payload); ts_phone = s.ts_ns
    else:                  count("bad_imu_length"); return

    θ     = predict_offset(ts_phone)                     # θ₀ + s·t, t = phone BOOTTIME
    stamp = to_ros(ts_phone + θ)

    if stamp > now_ns() + 5_000_000:                     # more than 5 ms in the future
        stamp = now_ns()                                 # clamp
        diag_warn_rate_limited("future_stamp_clamped")

    if stamp <= last_stamp:                              # never let time run backwards
        count("reordered")
        return
    last_stamp = stamp

    msg = Imu()
    msg.header.stamp    = to_time(stamp)                 # int32 sec + uint32 nanosec
    msg.header.frame_id = "phone_imu_link"

    if s.mode == 0:                                      # fused
        msg.orientation = Quaternion(x, y, z, w)
        msg.orientation_covariance[0] = 0.01             # tune from data
        msg.angular_velocity_covariance[0]        = -1   # not provided
        msg.linear_acceleration_covariance[0]     = -1   # not provided
    else:                                                # raw
        msg.orientation_covariance[0] = -1
        msg.angular_velocity = Vector3(gx, gy, gz)
        msg.linear_acceleration = Vector3(ax, ay, az)
        msg.angular_velocity_covariance[0]     = sigma_gyro**2
        msg.linear_acceleration_covariance[0]  = sigma_accel**2

    publish(msg)
```

Setting the first covariance element to `-1` is the ROS convention for "this field is not valid", and `robot_localization` respects it. Getting this wrong makes the EKF fuse zeros.

Two implementation requirements:

- `to_time(ns)` must normalize: `sec, nsec = divmod(ns, 1_000_000_000)` on **int64**, then assert `0 <= nsec < 1_000_000_000` and that `sec` fits in `int32`. Negative or oversized `nanosec` produces messages that silently fail to serialize.
- `last_stamp` is shared state. If the Zenoh callback and a ROS timer can run concurrently, guard it with a lock or use a single-threaded executor. A data race here produces occasional backwards stamps that are very hard to diagnose.

### 11.4 GPS ingest → `sensor_msgs/NavSatFix`

- Use `Location.getElapsedRealtimeNanos()` (BOOTTIME) as the sample time, not arrival time.
- `position_covariance` from `horizontal_accuracy` (variance = accuracy²); set `position_covariance[0]` and `[4]` (east/north) and set `position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN`. Zero the unused cells every message — these arrays are fixed-length and a stale cell is silent corruption.
- `status.service = NavSatStatus.SERVICE_GPS` and `status.status = STATUS_FIX` only when the fix is fresh and `accuracy` is finite; otherwise `STATUS_NO_FIX`.
- Bearing is not part of `NavSatFix`. If the global EKF needs heading, publish it separately as `std_msgs/Float64` on `/gps/bearing` (a ROS topic produced by the bridge, not a Zenoh key), and set `frame_id` conventions consistently.

### 11.5 Diagnostics

Publish `diagnostic_msgs/DiagnosticStatus` on `/phone/clock/status` at 1 Hz with key/values:

```
offset_ns, offset_std_ns, skew_ppm, path_delay_min_ns, path_delay_p99_ns,
last_burst_age_s, bursts_ok, bursts_failed, samples_in, samples_out,
drops_qos, reordered, future_clamped, sync_state ∈ {SYNCED, DEGRADED, UNSYNCED}
```

This is the observable that lets you prove or disprove the whole design on the real robot.

---

## 12. QoS

| Path | Reliability | History | Justification |
|---|---|---|---|
| Phone → bridge (`phone/sensors/imu`) | `BEST_EFFORT` | — | Already set by the app. A retransmitted IMU sample is useless |
| Bridge → ROS `/imu/data` | `BEST_EFFORT`, depth 10 | keep last | At 408 Hz, depth 10 is 24 ms; matching the phone prevents head-of-line blocking |
| Phone → bridge (`phone/sensors/gps`) | `RELIABLE` | — | Slow stream, worth retransmission |
| Bridge → ROS `/gps/fix` | `RELIABLE`, depth 5 | keep last | Matches `NavSatFix` consumers |
| Bridge → ROS `/gps/bearing` (optional) | `RELIABLE`, depth 5 | keep last | Derived heading, same cadence as GPS |
| Bridge → ROS `/phone/clock/status` | `RELIABLE`, depth 1 | keep last | Diagnostics must not be dropped |
| Clock query | default query QoS, 300 ms timeout | — | Set the timeout explicitly; defaults are far too long for a sync loop |

A `BEST_EFFORT` publisher and a `RELIABLE` subscriber do not connect under ROS 2 QoS rules. Verify the pairing — it fails silently.

---

## 13. Configuration and boilerplate

### 13.1 Topology on the Jetson

**Prerequisites — verify before anything else.** The current `docker/Dockerfile` is a plain `ros:jazzy-ros-base` image and does **not** install `rmw_zenoh_cpp`, `zenohd`, the Python `eclipse-zenoh` binding, or `pytest`. Also note that `app/ProjectZeRo/ingestion_node.py` opens its own Zenoh session in **peer** mode listening on `udp/tcp 0.0.0.0:7447`; running that node at the same time as a router will collide on the port. The dora path is being retired — do not run both.

```bash
ros2 pkg list | grep -E 'rmw_zenoh|ros2_control|robot_localization'   # confirms the RMW is installed
which zenohd                                                          # must exist for the router topology
python3 -c "import zenoh, struct; print(zenoh.__version__)"
```

If `zenohd` or `eclipse-zenoh` is missing, install them into the image first; without them nothing in §15 is reproducible.

Run **one** Zenoh router as the single hub. This avoids the classic failure where both `rmw_zenoh` and the bridge try to listen on 7447.

```bash
# terminal 1
zenohd --config /etc/zenoh/ntou_router.json5

# check nothing else took the port first
ss -lunp | grep 7447 ; ss -ltnp | grep 7447
```

`/etc/zenoh/ntou_router.json5`

```json5
{
  mode: "router",
  listen: { endpoints: ["tcp/0.0.0.0:7447"] },
  scouting: { multicast: { enabled: false } },
  // Wi-Fi MTU safety for JPEG frames; also caps per-link memory
  transport: {
    link: { tx: { batch_size: 65535, queue: { size: { link: 8, transport: 8 } } } }
  }
}
```

Phone: keep `mode = "client"`, `connect.endpoints = ["tcp/<jetson-ip>:7447"]`, multicast off — exactly as the current `ZenohTransport.connect()` builds it.

Bridge and `rmw_zenoh`: point both at the local router.

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=1
# Jazzy: supply a session config that connects (client) to the local router
export ZENOH_SESSION_CONFIG_URI=/etc/zenoh/rmw_client.json5
```

`/etc/zenoh/rmw_client.json5`

```json5
{
  mode: "client",
  connect: { endpoints: ["tcp/127.0.0.1:7447"] },
  scouting: { multicast: { enabled: false } }
}
```

Verify the actual environment-variable names for your installed `rmw_zenoh` version before relying on them *(verify)*; the behaviour to confirm is simply that `ros2 node list` sees the bridge and that port 7447 is owned by exactly one process.

### 13.2 Node parameters

```yaml
phone_zenoh_bridge:
  ros__parameters:
    use_sim_time: false
    zenoh_endpoint: "tcp/127.0.0.1:7447"
    zenoh_mode: "client"
    clock_query_key: "phone/clock/query"
    imu_key: "phone/sensors/imu"
    gps_key: "phone/sensors/gps"
    frame_id: "phone_imu_link"
    resync_period_s: 30.0
    health_probe_period_s: 10.0
    burst_size: 8
    burst_spacing_ms: 30
    query_timeout_ms: 300
    max_slew_ppm: 500
    future_clamp_ns: 5000000
    skew_window_s: 600.0
    sigma_gyro_rad_s: 0.01
    sigma_accel_m_s2: 0.05
    test_bias_ns: 0            # deliberate stamp bias for validation (#9); 0 in production
```

`test_bias_ns` is instrumentation, not a feature: it is the only way to prove the EKF sensitivity claim, and it must default to 0 and be rejected unless an explicit `allow_test_bias: true` is set.

### 13.3 Launch sketch

```python
# launch/phone_bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="phone_zenoh_bridge",
            executable="bridge",
            name="phone_zenoh_bridge",
            output="screen",
            parameters=["config/phone_bridge.yaml"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="phone_mount_tf",
            arguments=["0.45", "0.0", "0.42", "0.0", "0.30", "0.0",
                       "base_link", "phone_imu_link"],
        ),
    ])
```

The static transform must match the physical mount. `robot_localization` uses it to remove the lever-arm effect; a wrong value shows up as a yaw-dependent acceleration bias.

### 13.4 Language choice

- **Python (`rclpy` + `eclipse-zenoh`)**: fastest to build, adequate at 204–408 Hz if the hot path uses a precompiled `struct.Struct`, preallocated buffers, no `json`, and no per-sample logging. The GIL is fine when there is exactly one sensor thread per callback.
- **C++ (`rclcpp` + `zenoh-cpp`)**: choose this if measured drop rate or jitter is unacceptable, or once the camera stream is added.

Do not benchmark raw payload throughput and conclude either way — measure end-to-end timestamp jitter on the target robot.

---

## 14. Required Android changes

1. **Stamp from the sensor, not the callback.**
   ```kotlin
   val timestamp = event.timestamp          // CLOCK_BOOTTIME nanos
   ```
   and for GPS:
   ```kotlin
   val t = if (location.hasElapsedRealtimeNanos()) location.elapsedRealtimeNanos
           else SystemClock.elapsedRealtimeNanos()
   ```
2. **Timestamp each channel separately** in raw mode (`ts_accel_ns`, `ts_gyro_ns`) instead of one gyro-triggered value.
3. **Implement the clock client at session open, before the first publish:** burst of 8 queries, min-δ selection, store `θ₀` and later `s`.
4. **Add `boot_id` and `seq`** to the payload so the host can detect reboot, suspend gaps, reordering and duplicates.
5. **Re-sync every 30 s** and on events: session reconnect, network change, app resume from background, `ACTION_BOOT_COMPLETED`-style restarts.
6. **Never switch time bases mid-stream.** One affine map per session; corrections are slewed.
7. **Set `max_report_latency` to 0** (or the minimum) for the IMU sensors. Any batching latency is added to delivery but, thanks to `SensorEvent.timestamp`, no longer corrupts the stamp — it only delays it. Keep it at zero anyway to protect the EKF from stale bursts.

Optional but valuable: expose a `phone/clock/status` publisher with `(θ, s, δ_min, boot_id)` so the phone UI can show sync quality.

---

## 15. Validation plan on the real robot

Run these in order. Do not accept "it looks fine in RViz" as evidence.

**Ground truth is the hard part.** Measurements #4 and #5 compare the bridge's own numbers against the bridge's own numbers, so they prove *self-consistency*, not accuracy. To get an independent reference, run one baseline with the phone **tethered by USB/ethernet** (or on a wired AP) where one-way delay is under 0.2 ms; the residual measured there is the achievable floor, and the Wi-Fi run must be compared against it.

| # | Measurement | Method | Pass criterion |
|---|---|---|---|
| 0 | **Negative control** | Run the pipeline with sync disabled (raw phone stamps) and record EKF drift | Establishes the baseline the fix must beat; if this already passes, sync is unnecessary and the design should be simplified |
| 1 | Clock base sanity | Log `SensorEvent.timestamp`, `elapsedRealtimeNanos()`, `nanoTime()` together for 10 min (adb logcat) | Sensor and boot clocks agree within 1 ms and survive a screen-off period; `nanoTime` diverges after suspend. True deep sleep needs `dumpsys deviceidle force-idle`, not just screen-off |
| 2 | Drift (ppm) | Compare the phone boot clock against the host for 1 h stationary | Regression residuals bounded; record the fitted ppm |
| 3 | Path delay distribution | 1000 exchanges at 0 m, 1 m, 5 m; histogram `δ` | Gate on **δ_min**: p50 ≤ 2 ms, p99 ≤ 10 ms. Record `δ_p99` for information — it may legitimately reach 10–100 ms under Wi-Fi power save |
| 4 | Offset residual | Round-trip `θ̂` versus the one-way min-delay estimate, over ≥30 min | Difference p99 ≤ **5 ms** (≤2 ms acceptable only on the wired baseline). This is a consistency check, not accuracy |
| 5 | End-to-end stamp error | `now − stamp` logged by the bridge, against the wired-baseline reference | Median ≤ 1 ms, p99 ≤ 5 ms, **never negative**, no step larger than the declared slew |
| 6 | Rate and drops | `ros2 topic hz /imu/data` plus a phone-side sent counter | ≥ 203 Hz at 204 Hz mode, ≥ 406 Hz at 408 Hz mode; delivered/sent ratio ≥ 99.5 % |
| 7 | Ordering | Check strict monotonic `header.stamp` | Zero backward steps |
| 8 | TF health | Watch **node logs** (`--log-level debug` on the EKF) for `ExtrapolationException` / `LookupException`; use `view_frames` only for graph shape, it cannot detect time errors | Zero extrapolation warnings during a 10 min drive |
| 9 | EKF sensitivity | Add a `--bias-ns` parameter to the bridge to inject ±5 ms and ±10 ms deliberately | Yaw error < 0.5° at ±5 ms; ≥ 2× degradation at ±10 ms, confirming the §5 budget |
| 10 | Suspend behaviour | Force idle for 60 s, resume | Exactly one `clock_step` event, automatic re-sync, no permanent bias |
| 11 | Wi-Fi roam | Walk between two APs during streaming | `DEGRADED` then `SYNCED`; no stamp discontinuity except one declared slew |
| 12 | Soak | ≥ 1 h streaming, plus one overnight doze/resume cycle | No monotonic drift in `offset_ns` beyond the regression prediction; memory stable |

Useful commands:

```bash
ros2 topic hz /imu/data
ros2 topic delay /imu/data            # requires consistent clock domains
ros2 topic echo /phone/clock/status
ros2 topic info /imu/data --verbose   # check publisher/subscriber QoS actually matches
ros2 run tf2_tools view_frames
chronyc tracking                      # host NTP state; a stepped host clock is visible here
ss -lunp | grep 7447 ; ss -ltnp | grep 7447
```

### 15.1 Minimum automated test set

Most of the risk is in pure logic, which is cheap to test without hardware. Build these before the robot runs.

**Unit tests with a fake clock (highest value, no ROS, no Zenoh):**

- `θ̂` and `δ` from synthetic four-timestamp samples.
- Min-δ selection returns the lowest-delay sample, not the median.
- Marzullo intersection rejects a small-δ/huge-asymmetry sample.
- Skew regression recovers a known ppm from a synthetic offset ramp.
- Outlier gate: 3-consecutive-reject rule, and no step on a single bad burst.
- Step/slew thresholds at 100 µs and 1 ms boundaries.
- `to_time()` normalization for values near `1e9` ns and for negative deltas.
- Stamp clamp: future stamp is clamped, backwards stamp is counted and dropped.
- v1 (33 B) / v2 (42 B) dispatch, and rejection of 40/64-byte payloads.
- `boot_id` change triggers hard re-init.

**Loopback Zenoh test (two sessions in one process):** burst exchange end-to-end, handler latency, stale-reply rejection by `t1` echo, malformed-payload handling.

**ROS smoke test:** `ros2 topic info /imu/data --verbose` must show 1 publisher and 1 subscriber matched; a deliberate `BEST_EFFORT` publisher against a `RELIABLE` subscriber must show **no** match, proving the QoS check works.

```bash
docker compose up -d
docker exec -it ros2_jazzy_dev bash
cd /root/ros2_ws && colcon build --packages-select phone_zenoh_bridge
colcon test --packages-select phone_zenoh_bridge && colcon test-result --verbose
python3 -m pytest src/phone_zenoh_bridge/test -q
```

Success = `colcon` reports `0 errors, 0 failures` and `pytest` exits 0 with every test above present. The `phone_zenoh_bridge` package does not exist yet — it must be created before any of this is meaningful.

---

## 16. Risks and open questions

| Risk | Impact | Mitigation |
|---|---|---|
| Wi-Fi power save creates 100 ms delay sawtooth | Offset excursions of tens of ms | min-δ filtering; keep the radio awake while streaming; never trust high-δ bursts |
| Both `rmw_zenoh` and the bridge listen on 7447 | Total transport failure | Single `zenohd` router; explicit client configs; port check at startup |
| Python bridge at 408 Hz | Jitter, drops, GC | Precompiled struct, no JSON, no per-sample logging; fall back to C++ if measured |
| Phone suspends mid-stream | Stale offset, gaps | BOOTTIME base; event-triggered re-sync on wake; `boot_id` check |
| `SensorEvent.timestamp` deviates from spec on this vendor build | Systematic offset | Measurement #1 above; if it deviates, characterise it once |
| Host NTP step while streaming | Stamp discontinuity | Re-sample `k` per message; log `clock_step`; never silently absorb |
| `use_sim_time` accidentally true | Stamps stop advancing | Assert the parameter at startup and refuse to run otherwise |
| rmw_zenoh version drift changes key expressions | Bridge-to-ROS link breaks | Do not hand-craft rmw keys; use a real ROS node |

**Open questions that need evidence, not opinion:**

1. Actual ppm of the S23 oscillator — measurement #2.
2. Actual `δ` distribution on the target Wi-Fi at the intended operating distance — measurement #3.
3. Whether Android's sensor hub batches under `SENSOR_DELAY_FASTEST` with `max_report_latency = 0` on One UI — measurement #1.
4. Whether `zenoh-kotlin-android` 1.10.1 performs well enough for 8 sequential `get()` calls without stalling the streaming thread — needs a timing test.
5. Whether the EKF accepts a 5 ms residual without covariance inflation — measurement #9.

---

## 17. References

- Android: `SensorEvent.timestamp`, `SystemClock.elapsedRealtimeNanos()`, `System.nanoTime()`, sensor batching (`max_report_latency`) — Android platform reference.
- RFC 5905 — Network Time Protocol v4: offset/delay equations and the clock filter.
- `robot_localization`: `state_estimation_nodes` parameters (`transform_time_offset`, `transform_timeout`, `permit_corrected_publication`, `smooth_lagged_data`).
- `geometry2` / `tf2_ros::Buffer`: `lookupTransform`, `cache_time`, `ExtrapolationException`.
- `message_filters`: `ApproximateTime` policy, `slop`, `queue_size`.
- `ros2_control`: controller-manager clock documentation (`RCL_STEADY_TIME` vs `RCL_ROS_TIME`).
- `ros2/rmw_zenoh`, `docs/design.md`, branch `jazzy`: topic key expression `<domain_id>/<fqn>/<type_name>/<type_hash>` and `@ros2_lv/...` liveliness tokens.
- Eclipse Zenoh 1.10: queryable/query/reply API, `Timestamp` (NTP64 + ZenohId), `zenoh-kotlin` binding.

# Project ZeRo — Complete Architecture, Configurations & Experimental Findings
**Edge-to-Host Robotics Telemetry & Perception Pipeline (Samsung Galaxy S23 $\rightarrow$ Dora-RS / ROS 2 Jazzy)**

---

## 1. Project Overview & Core Philosophy
**Project ZeRo** turns a consumer flagship smartphone (**Samsung Galaxy S23**) into an industrial-grade, deterministic robotics sensor suite for an Unmanned Ground Vehicle (UGV). Rather than purchasing separate, expensive hardware modules (industrial USB global-shutter cameras, BNO085 IMUs, and u-blox GNSS receivers), Project ZeRo extracts high-frequency hardware streams directly from Android silicon and pipes them over low-latency wireless transport to a compute host (**Laptop prototyping environment via Dora-RS 1.0.1** and **Production NVIDIA Jetson Orin Nano running ROS 2 Jazzy**).

### Core Engineering Goals
1. **Zero-Allocation Edge Execution:** Prevent Android Garbage Collector (GC) pauses ("Stop-the-World" stutter) while streaming 200–400 Hz IMU telemetry and 30 FPS video.
2. **Zero-Bloat Network Transport:** Eliminate TCP/UDP head-of-line blocking and Wi-Fi buffer bloat using strict Zenoh Quality of Service (QoS) rules.
3. **Decoupled Wire Protocol:** Use raw Little-Endian C-structs (`APPLICATION_OCTET_STREAM`) over Zenoh so the Android app never breaks when ROS 2 distributions or RMW type hashes change.

---

## 2. Inside the Android App (`com.example.zenohtelemetry`)

### 2.1 Two-Stage Ground Control Station (GCS) UI (`MainActivity.kt`)
Built with Jetpack Compose, the UI uses a strict **Two-Stage State Machine** (`!isConnected` $\rightarrow$ `isConnected && !isStreaming` $\rightarrow$ `isStreaming`) so users cannot accidentally mutate network parameters mid-stream:

* **Header Bar:** Displays the `PROJECT ZERO` title stacked above a live connection indicator (`🔴 DISCONNECTED` / `🟢 CONNECTED`) locked to a single line (`maxLines = 1, softWrap = false`).
* **Stage 1 (Network Configuration Card):**
  * **Target Host IP:** Text field for the laptop or Jetson IP address (locked once connected).
  * **Advanced Options Toggle:** Collapsible card containing:
    * **Mode Selector:** `Client` vs. `Peer`.
    * **Transport Selector:** `TCP Unicast` vs. `UDP Unicast`.
    * **Terminal Toggle:** Shows or hides the real-time diagnostic console.
* **Stage 2 (Sensor Routing Card — Unlocked Only After Connection):**
  * **Camera Stream Toggle:** Enables the 30 FPS Main Wide lens pipeline.
  * **IMU Data (200Hz+) Toggle:** Reveals a vertical radio-button sub-menu:
    * `Fused (Quaternion)` — Hardware 6-axis Game Rotation Vector.
    * `Raw (Accel/Gyro)` — Calibrated Linear Acceleration + Gyroscope angular velocity.
  * **GPS Telemetry (15Hz) Toggle:** Enables the L1+L5 GNSS hardware listener.
  * **Decoupled Action Controls:** Primary green `START STREAMING` button stacked above a clean `Disconnect` text button. Once streaming starts, both collapse into a single red `STOP STREAMING & DISCONNECT` emergency halt button.
* **1 Hz Diagnostic Telemetry Loop:** A Compose `LaunchedEffect(isStreaming)` coroutine logs a clean summary line every 1,000 ms (`[STREAMING] Transport: TCP | Camera: 30Hz | IMU: 200Hz (Fused) | GPS: OFF`) instead of flooding the UI thread at 200 Hz.

---

### 2.2 Zenoh Network Transport Layer (`transport/ZenohTransport.kt`)
Manages the native `io.zenoh` session lifecycle and three dedicated publishers.

#### Why We Hardcoded Per-Sensor Reliability (Instead of a UI Dropdown)
Initially, we exposed a `BEST_EFFORT` vs. `RELIABLE` dropdown in the UI. Through testing, we proved that **reliability is a property of the sensor physics, not a user preference**:
* High-frequency IMU data (200–400 Hz) **must** be `BEST_EFFORT`. If a packet drops over Wi-Fi, requesting a retransmission 15 ms later is useless because 6 newer IMU readings have already been generated.
* Camera JPEG frames and slow GPS updates **benefit** from `RELIABLE` transport over TCP when paired with `CongestionControl.DROP`, preventing half-corrupted JPEG buffers.

#### The Final Zenoh QoS Matrix
| Stream | Key Expression | Encoding | Reliability | Congestion Control | Priority | Express Flag |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Camera** | `phone/sensors/camera` | `IMAGE_JPEG` | `RELIABLE` | `DROP` | `DATA_HIGH` | `true` |
| **IMU** | `phone/sensors/imu` | `APPLICATION_OCTET_STREAM` | `BEST_EFFORT` | `DROP` | `DATA_HIGH` | `true` |
| **GPS** | `phone/sensors/gps` | `APPLICATION_OCTET_STREAM` | `RELIABLE` | `DROP` | `DATA` | `false` |

#### Key Zenoh Settings Explained
1. **`scouting.multicast.enabled = false`:**
   * *Why:* When running Dora-RS on the host, Dora spins up its own internal Zenoh runtime. Leaving multicast scouting enabled caused the phone and Dora's internal daemon to collide on UDP multicast port `7446`, resulting in version mismatch errors and dropped sessions. Pure Unicast (`tcp/<IP>:7447` or `udp/<IP>:7447`) makes the connection deterministic.
2. **`CongestionControl.DROP` (Never `BLOCK`):**
   * *Why:* If the robot drives into a weak Wi-Fi zone, `BLOCK` forces the Android OS to queue camera frames in RAM, creating a 2–5 second video delay ("lag accumulation"). `DROP` forces Zenoh to immediately discard the frame at the network edge so the robot always acts on real-time vision.
3. **`express = true`:**
   * *Why:* By default, Zenoh batches small messages together for a few microseconds to optimize network throughput. Setting `express = true` disables batching and forces the OS socket to flush every single IMU and Camera packet onto the wire immediately, minimizing jitter for EKF/VIO integration.
4. **`Encoding.APPLICATION_OCTET_STREAM`:**
   * *Why:* Tells Zenoh and downstream nodes that the payload is an unformatted, raw binary C-struct memory block—preventing any UTF-8 string parsing overhead.

---

### 2.3 Camera Pipeline (`camera/CameraCaptureManager.kt`)
Uses the low-level Android `Camera2` API (`CameraDevice`, `CameraCaptureSession`, `ImageReader`) on a background thread.

* **Lens Selection:** Locked to Camera ID `"0"` (`MAIN_WIDE` rear sensor).
* **Strict `[30, 30]` AE Target FPS Range:**
  * *Why:* By default, Android auto-exposure (AE) dynamically drops the shutter speed in indoor lighting down to 12–15 FPS to gather more light, which causes motion blur and starves visual odometry (VIO) of frames. Locking `CONTROL_AE_TARGET_FPS_RANGE` to `[30, 30]` forces the ISP to boost ISO sensitivity instead of slowing the shutter, guaranteeing a rock-solid 30.0 Hz frame rate.
* **JPEG Compression Tuning (40% vs. 70% Findings):**
  * At 90–100% quality, a single frame spikes to 250–400 KB, causing massive UDP packet fragmentation (exceeding the 1,500-byte Wi-Fi MTU over 200 times per frame). If even one UDP fragment drops, the entire JPEG frame is discarded.
  * We tested **40% quality** (ultra-lightweight ~25 KB frames) and upgraded to **70% quality** (~55–85 KB frames), which provides crisp visual features for SLAM/VIO while surviving TCP/UDP Unicast without triggering congestion drops.
* **Metadata Attachment:** Every frame sends a JSON attachment (`{"seq": frameSequence, "media_ts": System.nanoTime()}`) alongside the raw JPEG bytes so the host can calculate exact frame drops and network transport latency.

---

### 2.4 Zero-Allocation Sensor Engine (`sensors/TelemetryManager.kt`)
Streams IMU and GPS data from a dedicated `HandlerThread("TelemetryBackground")` so sensor interrupts never block the Compose UI thread.

#### Why Zero-Allocation `ByteBuffer` Matters
If you serialize IMU readings into JSON strings at 400 Hz, you allocate thousands of temporary `String` and `HashMap` objects per second. Every few seconds, the Android JVM Garbage Collector halts the process for 10–30 ms to free heap memory, injecting catastrophic timing jitter into the IMU stream.
* **Our Solution:** We allocate `imuByteArray = ByteArray(33)` and `gpsByteArray = ByteArray(48)` **once** when the class loads, wrapped in Little-Endian `ByteBuffer` views (`ByteOrder.LITTLE_ENDIAN`). On every hardware interrupt, we call `imuBuffer.clear()`, overwrite the exact same 33 bytes in RAM, and pass the pointer to Zenoh. **Zero heap allocations. Zero GC pauses.**

#### Binary C-Struct Layouts
1. **Current 33-Byte IMU Struct (`<qB6f`):**
   * `int64` (8 bytes): Timestamp (`ns`)
   * `uint8` (1 byte): Mode flag (`0` = Fused Quaternion, `1` = Raw Accel + Gyro)
   * `6 x float32` (24 bytes):
     * Mode `0`: `[q_x, q_y, q_z, q_w, 0.0f, 0.0f]` (padded with two zeros to maintain fixed 33-byte alignment)
     * Mode `1`: `[accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]`
2. **Current 48-Byte GPS Struct (`<q3d4f`):**
   * `int64` (8 bytes): Timestamp (`ns`)
   * `3 x float64` (24 bytes): `latitude`, `longitude`, `altitude` (Doubles are mandatory for Lat/Lon; 32-bit floats lose sub-meter GPS precision!)
   * `4 x float32` (16 bytes): `speed` (m/s), `horizontal_accuracy` (m), `bearing` (deg from True North), `bearing_accuracy` (deg)

---

## 3. Experimental Findings, Bugs & Post-Mortems

### 3.1 The 408 Hz vs. 204 Hz IMU Discovery
When we requested `SensorManager.SENSOR_DELAY_FASTEST` on the Samsung S23:
* **`TYPE_GAME_ROTATION_VECTOR` (Fused Mode):** Streamed at a steady **~204.4 Hz**.
* **`TYPE_LINEAR_ACCELERATION` + `TYPE_GYROSCOPE` (Raw Mode):** Streamed at **~408.7 Hz**.
* *Why:* The S23's internal Bosch/STMicro IMU hardware samples raw MEMS registers at ~400–500 Hz, while the Qualcomm Sensor Core decimates the fused quaternion output to 200 Hz.

### 3.2 The API 31+ `HIGH_SAMPLING_RATE_SENSORS` Crash
* **What Happened:** Enabling IMU streaming immediately crashed the app with:
  `java.lang.SecurityException: To use the sampling rate of 0 microseconds, app needs to declare the normal permission HIGH_SAMPLING_RATE_SENSORS.`
* **Why:** Starting in Android 12 (API 31), Google restricted >200 Hz motion sensor access because malicious apps could use 400 Hz accelerometer vibrations to infer touchscreen keystrokes.
* **The Fix:** Added `<uses-permission android:name="android.permission.HIGH_SAMPLING_RATE_SENSORS" />` to `AndroidManifest.xml` (a normal permission granted automatically at install time without a popup).

### 3.3 The Silent GPS Permission & Indoor `0.0 Hz` Trap
* **Bug 1 (Missing Manifest Tags):** Calling `permissionLauncher.launch(ACCESS_FINE_LOCATION)` in Kotlin silently returned `false` without showing a popup because `<uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" />` was missing from `AndroidManifest.xml`.
* **Bug 2 (Location Toggle Off):** Even with app permission granted, if the user's swipe-down Location toggle is off, `requestLocationUpdates` either throws an exception or silently stalls. We added `locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)` as a hardware guard.
* **Finding 3 (Why GPS Shows 0.0 Hz Indoors While Amazon Works):**
  * Consumer apps (Amazon, Uber) use `FusedLocationProvider` / `NETWORK_PROVIDER`, which triangulates Wi-Fi routers and cell towers indoors. However, Wi-Fi triangulation snaps by 20–50 meters when hopping routers—which would cause a UGV's EKF to violently jerk the steering wheels.
  * By binding strictly to `LocationManager.GPS_PROVIDER`, we force the S23 to use only L1+L5 satellite microwaves. Indoors, satellite lock is impossible (`0.0 Hz`). Outdoors under open sky, after a 15–45 second **Time To First Fix (TTFF)**, it locks onto 15+ satellites with **1–2 meter accuracy** and computes true Doppler/trajectory **Bearing** once moving over 3–5 meters.

### 3.4 The "Zero Yaw Drift" Illusion (ZUPT)
* **Observation:** When testing `TYPE_GAME_ROTATION_VECTOR` (which disables the magnetometer) on the 3D visualizer (`imu_3d_viewer.py`), the 3D wedge sat completely rock-solid on the desk with `0.000` drift.
* **The Physics Explanation:** Qualcomm's sensor core runs **Zero-Velocity Updating (ZUPT)**. When the accelerometer detects zero vibration variance, it knows the phone is sitting on a table and clamps the gyroscope integration bias to zero. Once mounted on a vibrating UGV chassis in motion, ZUPT disengages and the 6-axis yaw drifts at **0.5°–2°/min** (up to **15°–30°/min** under heavy vibration)—proving why VIO and wheel encoders are mandatory to anchor yaw.

### 3.5 The Dora-RS "30.3 GB Memory" Illusion
* **Observation:** Running `dora list` showed `Memory: 30.3 GB` on the active dataflow.
* **Explanation:** Dora-RS uses **Apache Arrow Zero-Copy Shared Memory (`mmap`)**. The daemon pre-reserves a large virtual memory address space with the Linux kernel so nodes can pass raw C-pointers without asking the OS to allocate RAM on every frame. Physical RAM consumption remained under a few hundred megabytes.

---

## 4. UGV Robotics & Sensor Fusion Blueprint (Jetson Orin Nano)

### 4.1 Why We Dropped the Magnetometer
Indoors, steel rebar and power lines warp magnetic fields; on the UGV, high-current drive wires create electromagnetic interference (EMI). Furthermore, Android's background compass calibration expects human "figure-8" hand motion and breaks when bolted flat to a robot. We use 6-axis IMU (`GAME_ROTATION_VECTOR` + Raw Accel/Gyro) exclusively.

### 4.2 Solving the Un-Encoded IBT-2 Steering Problem
Our UGV has a wheel speed encoder on the drive motor, but uses an **IBT-2 H-Bridge** for steering **without a steering angle encoder**.
* **The EKF Rule:** Never estimate turning radius from raw PWM steering commands (wheel slip and mechanical backlash will destroy odometry).
* **The Fix:** In `robot_localization`, configure the Wheel Odometry input matrix to fuse **only Linear $X$ Velocity ($v_x$)**. Configure the Phone IMU + Phone VIO to handle **100% of Yaw ($\psi$) and Angular Velocity ($\omega_z$)**.

### 4.3 Dashcam Mounting & Lever-Arm Compensation
Mounting the phone at the front of the robot (~40–50 cm high, tilted 15°–20° down to see floor texture for VIO) means the IMU is offset from the robot's center of rotation (`base_link`).
* When the UGV turns, the front-mounted phone experiences **centripetal and tangential acceleration** ($a = \alpha \times r + \omega \times (\omega \times r)$).
* You **must** broadcast a `tf2` `static_transform_publisher` from `base_link` to `phone_imu_link` with the exact physical $(x, y, z, \text{roll}, \text{pitch}, \text{yaw})$ measurements so the EKF mathematically subtracts the lever-arm effect.

### 4.4 Dual-EKF Architecture (Indoor $\leftrightarrow$ Outdoor Transition)
1. **Local EKF (`odom` frame):** Fuses Wheel Encoder ($v_x$) + Phone 6-Axis IMU + Phone VIO + 2D LiDAR SLAM indoors. Never touches GPS. Outputs smooth, continuous local coordinates so steering control never jerks.
2. **Global EKF (`map` frame):** Fuses Local EKF + S23 Dual-Band GPS (`NavSatFix` + `bearing`) outdoors. Absorbs 1–3 meter GPS coordinate jumps smoothly in the background map transformation (`map` $\rightarrow$ `odom`).
3. **Environment Switching:** Detect indoor vs. outdoor state automatically via GPS satellite count / horizontal accuracy (`accuracy < 4.0m`) and VIO tracking state. Downgrade the 2D LiDAR from SLAM localization to simple obstacle avoidance outdoors (where sunlight IR noise and lack of walls degrade 2D laser scans).

---

## 5. Why We Chose Pure ROS 2 Jazzy Over Dora-RS on the Jetson Orin Nano

### 5.1 Dora-RS vs. ROS 2 Zero-Copy Reality
* **Why Dora claims "17x faster than ROS 2":** That benchmark compares **Python-to-Python (`rclpy` vs. Dora Python)** sending large payloads (`>4 KB` HD images). `rclpy` copies and serializes data through C++ DDS layers, whereas Dora shares Apache Arrow RAM pointers across Python processes.
* **Small Payloads (`<4 KB`):** For our 33–57 byte IMU packets, Dora does not even use shared memory; latency is ~0.5–2 ms in both systems.
* **ROS 2 Native Zero-Copy (`rclcpp` Composable Containers):** In C++ (`rclcpp`), ROS 2 achieves true zero-copy by loading nodes (`slam_toolbox`, `nav2`, `robot_localization`) as shared libraries into a single process (`component_container`) and passing `std::unique_ptr` memory pointers directly in RAM—or across C++ processes via Shared Memory (`LoanedMessage`).

### 5.2 Final Orin Nano Bridge Architecture (`phone_zenoh_bridge`)
Running Dora-RS alongside ROS 2 Jazzy on an 8 GB Orin Nano wastes ~500 MB of RAM and runs two competing Zenoh daemons. Instead, we use **Option 3**:
1. **Phone:** Continues sending ultra-fast, zero-allocation Little-Endian binary C-structs over Zenoh (`phone/sensors/imu`, `phone/sensors/vio`, `phone/sensors/gps`).
2. **Jetson Orin Nano:** Runs one lightweight (~45 MB) Python ROS 2 node (`phone_zenoh_bridge`) that listens on `tcp/0.0.0.0:7447`, unpacks the binary structs via `struct.unpack`, applies a **sliding-window minimum clock-offset filter** to align Android hardware timestamps (`SensorEvent.timestamp`) with the Jetson's `CLOCK_REALTIME`, and publishes standard ROS 2 topics (`/imu/data`, `/vio/odom`, `/gps/fix`).

---

## 6. Quick-Reference Command Cheat Sheet

### Launch Android Studio from Terminal
```bash
/opt/android-studio/bin/studio.sh ~/AndroidStudioProjects/ZenohTelemetry &
```

### Run Host Prototyping Pipeline (Dora-RS 1.0.1)
```bash
# Terminal 1: Boot daemon and start dataflow in background
dora up
dora start dataflow.yml --detach

# Terminal 2: View Camera FPS, Latency, and Drop metrics
dora logs --node viewer

# Terminal 3: View 200-400Hz IMU & GPS live frequency and unpacked values
dora logs --node telemetry_viewer

# Stop active dataflow and clean up zombies
dora stop
dora destroy
```
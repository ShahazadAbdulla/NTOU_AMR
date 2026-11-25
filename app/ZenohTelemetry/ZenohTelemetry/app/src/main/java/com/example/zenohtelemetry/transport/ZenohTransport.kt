package com.example.zenohtelemetry.transport

import android.os.SystemClock
import android.util.Log
import io.zenoh.Config
import io.zenoh.Session
import io.zenoh.Zenoh
import io.zenoh.bytes.Encoding
import io.zenoh.bytes.IntoZBytes
import io.zenoh.bytes.ZBytes
import io.zenoh.handlers.Callback
import io.zenoh.keyexpr.intoKeyExpr
import io.zenoh.pubsub.Publisher
import io.zenoh.qos.CongestionControl
import io.zenoh.qos.Priority
import io.zenoh.qos.QoS
import io.zenoh.qos.Reliability
import io.zenoh.pubsub.Subscriber
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.Locale
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong

data class TelemetryStats(
    val camSent: Long,
    val camDrops: Long,
    val imuSent: Long,
    val imuDrops: Long,
    val gpsSent: Long,
    val gpsDrops: Long
)

data class ClockSample(val t1: Long, val theta: Long, val delta: Long)

class ZenohTransport {
    private var session: Session? = null
    private var camPublisher: Publisher? = null
    private var imuPublisher: Publisher? = null
    private var gpsPublisher: Publisher? = null
    private var clockReportPublisher: Publisher? = null

    private var currentHostIp: String? = null
    private val clockSeq = AtomicLong(0)
    private val bootId: Long = ((System.currentTimeMillis() - SystemClock.elapsedRealtime()) / 1000L)

    var isConnected = false
        private set

    // Performance metrics / atomic counters
    private val camSent = AtomicLong(0)
    private val camDrops = AtomicLong(0)
    private val imuSent = AtomicLong(0)
    private val imuDrops = AtomicLong(0)
    private val gpsSent = AtomicLong(0)
    private val gpsDrops = AtomicLong(0)

    private var lastErrorLogTime = 0L

    fun getAndResetStats(): TelemetryStats {
        return TelemetryStats(
            camSent = camSent.getAndSet(0),
            camDrops = camDrops.getAndSet(0),
            imuSent = imuSent.getAndSet(0),
            imuDrops = imuDrops.getAndSet(0),
            gpsSent = gpsSent.getAndSet(0),
            gpsDrops = gpsDrops.getAndSet(0)
        )
    }

    fun connect(endpoint: String, mode: String): String {
        return try {
            disconnect()

            val config = Config.default()
            config.insertJson5("mode", "\"$mode\"").getOrThrow()
            config.insertJson5("connect/endpoints", "[\"$endpoint\"]").getOrThrow()
            config.insertJson5("scouting/multicast/enabled", "false").getOrThrow()

            Log.d("ZenohTransport", "Opening Zenoh session to $endpoint in $mode mode")
            session = Zenoh.open(config).getOrThrow()

            // 1. Camera Stream: BEST_EFFORT + DROP (Latest-frame priority on Wi-Fi)
            val camKey = "phone/sensors/camera".intoKeyExpr().getOrThrow()
            val camQos = QoS(congestionControl = CongestionControl.DROP, priority = Priority.DATA_HIGH, express = true)
            camPublisher = session!!.declarePublisher(camKey, qos = camQos, reliability = Reliability.BEST_EFFORT).getOrThrow()

            // 2. IMU Telemetry (200Hz): BEST_EFFORT + DROP
            val imuKey = "phone/sensors/imu".intoKeyExpr().getOrThrow()
            val imuQos = QoS(congestionControl = CongestionControl.DROP, priority = Priority.DATA_HIGH, express = true)
            imuPublisher = session!!.declarePublisher(imuKey, qos = imuQos, reliability = Reliability.BEST_EFFORT).getOrThrow()

            // 3. GPS Telemetry (15Hz): RELIABLE + DROP
            val gpsKey = "phone/sensors/gps".intoKeyExpr().getOrThrow()
            val gpsQos = QoS(congestionControl = CongestionControl.DROP, priority = Priority.DATA)
            gpsPublisher = session!!.declarePublisher(gpsKey, qos = gpsQos, reliability = Reliability.RELIABLE).getOrThrow()

            // 4. Clock Sync Report Publisher (32-byte burst summary): RELIABLE
            val reportKey = "phone/clock/report".intoKeyExpr().getOrThrow()
            val reportQos = QoS(congestionControl = CongestionControl.BLOCK, priority = Priority.DATA_HIGH, express = true)
            clockReportPublisher = session!!.declarePublisher(reportKey, qos = reportQos, reliability = Reliability.RELIABLE).getOrThrow()

            currentHostIp = endpoint.substringAfter("tcp/").substringAfter("udp/").substringBefore(":")
            isConnected = true
            "SUCCESS: Connected [$mode] to $endpoint"
        } catch (e: Exception) {
            Log.e("ZenohTransport", "Connection error: ${e.message}", e)
            isConnected = false
            "FAILED: ${e.message}"
        }
    }

    fun publishCameraFrame(jpegBytes: ByteArray, seq: Long, timestampNs: Long) {
        if (!isConnected) return
        try {
            val payload = ByteBuffer.allocate(16 + jpegBytes.size)
                .order(ByteOrder.LITTLE_ENDIAN)
                .putLong(timestampNs)
                .putLong(seq)
                .put(jpegBytes)
                .array()

            camPublisher?.put(
                ZBytes.from(payload),
                Encoding.APPLICATION_OCTET_STREAM
            )
            camSent.incrementAndGet()
        } catch (e: Exception) {
            camDrops.incrementAndGet()
            logThrottledError("Camera frame drop: ${e.message}")
        }
    }

    fun publishImuData(payload: ByteArray) {
        if (!isConnected) return
        try {
            imuPublisher?.put(ZBytes.from(payload), Encoding.APPLICATION_OCTET_STREAM)
            imuSent.incrementAndGet()
        } catch (e: Exception) {
            imuDrops.incrementAndGet()
            logThrottledError("IMU drop: ${e.message}")
        }
    }

    fun publishGpsData(payload: ByteArray) {
        if (!isConnected) return
        try {
            gpsPublisher?.put(ZBytes.from(payload), Encoding.APPLICATION_OCTET_STREAM)
            gpsSent.incrementAndGet()
        } catch (e: Exception) {
            gpsDrops.incrementAndGet()
            logThrottledError("GPS drop: ${e.message}")
        }
    }

    private fun logThrottledError(message: String) {
        val now = System.currentTimeMillis()
        if (now - lastErrorLogTime > 1000) {
            lastErrorLogTime = now
            Log.e("ZenohTransport", message)
        }
    }

    /**
     * True only when the session has at least one live router or peer link.
     * [Zenoh.open] is asynchronous and returns before any handshake completes, so
     * [isConnected] alone is not proof of a working link — this queries the session's actual
     * connected routers/peers.
     */
    fun hasActiveLink(): Boolean {
        val current = session ?: return false
        return runCatching {
            val routers = current.info().routersZid().getOrNull().orEmpty()
            val peers = current.info().peersZid().getOrNull().orEmpty()
            routers.isNotEmpty() || peers.isNotEmpty()
        }.getOrDefault(false)
    }

    /**
     * Blocks until a link is up or [timeoutMs] elapses. Call from a background dispatcher.
     */
    fun awaitActiveLink(timeoutMs: Long = 3_000): Boolean {
        val deadline = System.currentTimeMillis() + timeoutMs
        while (System.currentTimeMillis() < deadline) {
            if (hasActiveLink()) return true
            Thread.sleep(150)
        }
        return hasActiveLink()
    }

    fun runClockSyncBurst(burstSize: Int = 8, spacingMs: Long = 30L): String {
        val targetIp = currentHostIp ?: return "SYNC SKIPPED: No host IP"
        val samples = mutableListOf<ClockSample>()
        val queryBuf = ByteBuffer.allocate(28).order(ByteOrder.LITTLE_ENDIAN)
        val replyBytes = ByteArray(48)
        val replyBuf = ByteBuffer.wrap(replyBytes).order(ByteOrder.LITTLE_ENDIAN)

        try {
            DatagramSocket().use { socket ->
                socket.soTimeout = 300
                val address = InetAddress.getByName(targetIp)

                for (i in 0 until burstSize) {
                    val seq = clockSeq.incrementAndGet()
                    val t1 = SystemClock.elapsedRealtimeNanos()
                    queryBuf.clear()
                    queryBuf.putLong(t1).putLong(bootId).putLong(seq).putInt(1)

                    try {
                        val sendPkt = DatagramPacket(queryBuf.array(), 28, address, 7448)
                        socket.send(sendPkt)

                        val recvPkt = DatagramPacket(replyBytes, 48)
                        socket.receive(recvPkt)
                        val t4 = SystemClock.elapsedRealtimeNanos()

                        if (recvPkt.length == 48) {
                            replyBuf.clear()
                            val t1Echo = replyBuf.long
                            val t2 = replyBuf.long
                            val t3 = replyBuf.long
                            val delta = (t4 - t1) - (t3 - t2)
                            if (t1Echo == t1 && t4 > t1 && t3 >= t2 && delta in 1..250_000_000L) {
                                val theta = ((t2 - t1) + (t3 - t4)) / 2L
                                samples.add(ClockSample(t1, theta, delta))
                            }
                        }
                    } catch (_: Exception) {
                        // Timeout or packet drop on single sample; continue burst
                    }

                    if (i < burstSize - 1) Thread.sleep(spacingMs)
                }
            }
        } catch (e: Exception) {
            return "SYNC SKIPPED: ${e.message}"
        }

        if (samples.size < (burstSize / 2) + 1 || !marzulloAgreement(samples)) {
            return "SYNC DEGRADED: Valid=${samples.size}/$burstSize"
        }

        val best = samples.minByOrNull { it.delta } ?: return "SYNC DEGRADED: Selection failed"
        val reportBuf = ByteBuffer.allocate(32).order(ByteOrder.LITTLE_ENDIAN)
            .putLong(bootId)
            .putLong(best.t1)
            .putLong(best.theta)
            .putLong(best.delta)

        try {
            clockReportPublisher?.put(ZBytes.from(reportBuf.array()), Encoding.APPLICATION_OCTET_STREAM)
        } catch (e: Exception) {
            Log.e("ZenohTransport", "Failed to publish clock sync report: ${e.message}")
        }

        val rttMs = best.delta / 1e6
        return String.format(Locale.US, "CLOCK SYNCED: RTT=%.2f ms ( offset=%d ns )", rttMs, best.theta)
    }

    private fun marzulloAgreement(samples: List<ClockSample>): Boolean {
        val events = mutableListOf<Pair<Long, Int>>()
        for (s in samples) {
            val half = s.delta / 2L
            events.add(Pair(s.theta - half, -1))
            events.add(Pair(s.theta + half, +1))
        }
        events.sortWith(compareBy({ it.first }, { it.second }))
        var count = 0
        var maxOverlap = 0
        for (e in events) {
            if (e.second == -1) {
                count++
                if (count > maxOverlap) maxOverlap = count
            } else {
                count--
            }
        }
        return maxOverlap > (samples.size / 2)
    }

    fun disconnect() {
        runCatching { camPublisher?.close() }
        runCatching { imuPublisher?.close() }
        runCatching { gpsPublisher?.close() }
        runCatching { clockReportPublisher?.close() }
        runCatching { session?.close() }
        camPublisher = null
        imuPublisher = null
        gpsPublisher = null
        clockReportPublisher = null
        session = null
        isConnected = false
    }
}

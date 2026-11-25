package com.example.zenohtelemetry.transport

import android.util.Log
import io.zenoh.Config
import io.zenoh.Session
import io.zenoh.Zenoh
import io.zenoh.bytes.Encoding
import io.zenoh.bytes.ZBytes
import io.zenoh.keyexpr.intoKeyExpr
import io.zenoh.pubsub.Publisher
import io.zenoh.qos.CongestionControl
import io.zenoh.qos.Priority
import io.zenoh.qos.QoS
import io.zenoh.qos.Reliability

class ZenohTransport {
    private var session: Session? = null
    private var camPublisher: Publisher? = null
    private var imuPublisher: Publisher? = null
    private var gpsPublisher: Publisher? = null

    var isConnected = false
        private set

    fun connect(endpoint: String, mode: String): String {
        return try {
            disconnect()

            // Pure Unicast configuration. Multicast is explicitly disabled in all modes.
            val jsonConfig = """
            {
                "mode": "$mode",
                "connect": { "endpoints": ["$endpoint"] },
                "scouting": { "multicast": { "enabled": false } }
            }
            """.trimIndent()

            val config = Config.fromJson(jsonConfig).getOrThrow()
            session = Zenoh.open(config).getOrThrow()

            // 1. Camera Stream: RELIABLE + DROP
            val camKey = "phone/sensors/camera".intoKeyExpr().getOrThrow()
            val camQos = QoS(congestionControl = CongestionControl.DROP, priority = Priority.DATA_HIGH, express = true)
            camPublisher = session!!.declarePublisher(camKey, qos = camQos, reliability = Reliability.RELIABLE).getOrThrow()

            // 2. IMU Telemetry (200Hz): BEST_EFFORT + DROP
            val imuKey = "phone/sensors/imu".intoKeyExpr().getOrThrow()
            val imuQos = QoS(congestionControl = CongestionControl.DROP, priority = Priority.DATA_HIGH, express = true)
            imuPublisher = session!!.declarePublisher(imuKey, qos = imuQos, reliability = Reliability.BEST_EFFORT).getOrThrow()

            // 3. GPS Telemetry (15Hz): RELIABLE + DROP
            val gpsKey = "phone/sensors/gps".intoKeyExpr().getOrThrow()
            val gpsQos = QoS(congestionControl = CongestionControl.DROP, priority = Priority.DATA)
            gpsPublisher = session!!.declarePublisher(gpsKey, qos = gpsQos, reliability = Reliability.RELIABLE).getOrThrow()

            isConnected = true
            "SUCCESS: Connected [$mode] to $endpoint"
        } catch (e: Exception) {
            isConnected = false
            "FAILED: ${e.message}"
        }
    }

    fun publishCameraFrame(jpegBytes: ByteArray, seq: Long) {
        if (!isConnected) return
        try {
            val meta = """{"seq":$seq,"media_ts":${System.nanoTime()}}"""
            camPublisher?.put(
                ZBytes.from(jpegBytes),
                Encoding.IMAGE_JPEG,
                ZBytes.from(meta)
            )
        } catch (e: Exception) {
            Log.e("ZenohTransport", "Frame drop: ${e.message}")
        }
    }

    fun publishImuData(payload: ByteArray) {
        if (!isConnected) return
        try {
            // APP_OCTET_STREAM indicates raw binary C-struct data
            imuPublisher?.put(ZBytes.from(payload), Encoding.APPLICATION_OCTET_STREAM)
        } catch (e: Exception) {
            Log.e("ZenohTransport", "IMU drop: ${e.message}")
        }
    }

    fun publishGpsData(payload: ByteArray) {
        if (!isConnected) return
        try {
            gpsPublisher?.put(ZBytes.from(payload), Encoding.APPLICATION_OCTET_STREAM)
        } catch (e: Exception) {
            Log.e("ZenohTransport", "GPS drop: ${e.message}")
        }
    }

    fun disconnect() {
        runCatching { camPublisher?.close() }
        runCatching { imuPublisher?.close() }
        runCatching { gpsPublisher?.close() }
        runCatching { session?.close() }
        camPublisher = null
        imuPublisher = null
        gpsPublisher = null
        session = null
        isConnected = false
    }
}
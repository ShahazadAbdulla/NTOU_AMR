package com.example.zenohtelemetry.sensors

import android.annotation.SuppressLint
import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import java.nio.ByteBuffer
import java.nio.ByteOrder
import android.os.SystemClock

class TelemetryManager(
    context: Context,
    private val onImuData: (ByteArray) -> Unit,
    private val onGpsData: (ByteArray) -> Unit
) : SensorEventListener, LocationListener {

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager

    private var backgroundThread: HandlerThread? = null
    private var backgroundHandler: Handler? = null

    private val latestQuat = floatArrayOf(0f, 0f, 0f, 1f)
    private val latestAccel = floatArrayOf(0f, 0f, 0f)

    // --- ZERO-ALLOCATION BUFFERS (LITTLE-ENDIAN) ---
    // Universal IMU C-Struct (v3): int64 (timestamp) + int8 (mode=2) + 4x float32 (Quat) + 3x float32 (Gyro) + 3x float32 (Accel) = 49 Bytes
    private val imuByteArray = ByteArray(49)
    private val imuBuffer = ByteBuffer.wrap(imuByteArray).order(ByteOrder.LITTLE_ENDIAN)

    // GPS C-Struct: int64 (timestamp) + 3x float64 (lat,lon,alt) + 2x float32 (speed,acc) = 48 Bytes
    private val gpsByteArray = ByteArray(48)
    private val gpsBuffer = ByteBuffer.wrap(gpsByteArray).order(ByteOrder.LITTLE_ENDIAN)

    @SuppressLint("MissingPermission")
    fun startSensors(enableImu: Boolean, imuRateHz: String = "200 Hz", enableGps: Boolean) {
        backgroundThread = HandlerThread("TelemetryBackground").also { it.start() }
        backgroundHandler = Handler(backgroundThread!!.looper)

        if (enableImu) {
            val samplingPeriodUs = when (imuRateHz) {
                "50 Hz" -> 20_000
                "100 Hz" -> 10_000
                "200 Hz" -> 5_000
                "400 Hz" -> SensorManager.SENSOR_DELAY_FASTEST
                else -> 5_000
            }

            val gameRot = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
            val linearAccel = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
            val gyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

            sensorManager.registerListener(this, gameRot, samplingPeriodUs, backgroundHandler)
            sensorManager.registerListener(this, linearAccel, samplingPeriodUs, backgroundHandler)
            sensorManager.registerListener(this, gyro, samplingPeriodUs, backgroundHandler)
            Log.d("TelemetryManager", "Started Universal IMU at $imuRateHz ($samplingPeriodUs us)")
        }

        if (enableGps) {
            // Request absolute fastest GPS refresh (0ms interval, 0m distance)
            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0L, 0f, this, backgroundThread!!.looper)
            Log.d("TelemetryManager", "Started GPS Telemetry")
        }
    }

    fun stopSensors() {
        sensorManager.unregisterListener(this)
        locationManager.removeUpdates(this)

        backgroundThread?.quitSafely()
        try {
            backgroundThread?.join()
            backgroundThread = null
            backgroundHandler = null
        } catch (e: Exception) {
            Log.e("TelemetryManager", "Error stopping thread: ${e.message}")
        }
    }

    // --- IMU HIGH-FREQUENCY LISTENER ---
    override fun onSensorChanged(event: SensorEvent) {
        val timestamp = event.timestamp // Hardware Sensor Hub CLOCK_BOOTTIME ns

        when (event.sensor.type) {
            Sensor.TYPE_GAME_ROTATION_VECTOR -> {
                latestQuat[0] = event.values[0]
                latestQuat[1] = event.values[1]
                latestQuat[2] = event.values[2]
                latestQuat[3] = if (event.values.size > 3) event.values[3] else 1.0f
            }
            Sensor.TYPE_LINEAR_ACCELERATION -> {
                latestAccel[0] = event.values[0]
                latestAccel[1] = event.values[1]
                latestAccel[2] = event.values[2]
            }
            Sensor.TYPE_GYROSCOPE -> {
                // Gyroscope acts as master clock trigger (~200Hz)
                imuBuffer.clear()
                imuBuffer.putLong(timestamp)
                imuBuffer.put(2.toByte()) // Mode 2 = Universal (Quat + Gyro + Accel)
                imuBuffer.putFloat(latestQuat[0]).putFloat(latestQuat[1]).putFloat(latestQuat[2]).putFloat(latestQuat[3])
                imuBuffer.putFloat(event.values[0]).putFloat(event.values[1]).putFloat(event.values[2])
                imuBuffer.putFloat(latestAccel[0]).putFloat(latestAccel[1]).putFloat(latestAccel[2])

                onImuData(imuByteArray)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    // --- GPS LISTENER ---
    override fun onLocationChanged(location: Location) {
        gpsBuffer.clear()
        val gpsTimestamp = if (location.elapsedRealtimeNanos != 0L) location.elapsedRealtimeNanos else SystemClock.elapsedRealtimeNanos()
        gpsBuffer.putLong(gpsTimestamp)
        gpsBuffer.putDouble(location.latitude)
        gpsBuffer.putDouble(location.longitude)
        gpsBuffer.putDouble(location.altitude)
        gpsBuffer.putFloat(location.speed)
        gpsBuffer.putFloat(location.accuracy)
        gpsBuffer.putFloat(location.bearing)
        gpsBuffer.putFloat(if (location.hasBearingAccuracy()) location.bearingAccuracyDegrees else 0.0f)

        onGpsData(gpsByteArray)
    }
}
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

class TelemetryManager(
    context: Context,
    private val onImuData: (ByteArray) -> Unit,
    private val onGpsData: (ByteArray) -> Unit
) : SensorEventListener, LocationListener {

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val locationManager = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager

    private var backgroundThread: HandlerThread? = null
    private var backgroundHandler: Handler? = null

    private var isRawMode = false
    private val latestAccel = FloatArray(3)
    private val latestGyro = FloatArray(3)

    // --- ZERO-ALLOCATION BUFFERS (LITTLE-ENDIAN) ---
    // IMU C-Struct: int64 (timestamp) + int8 (mode) + 6x float32 = 33 Bytes
    private val imuByteArray = ByteArray(33)
    private val imuBuffer = ByteBuffer.wrap(imuByteArray).order(ByteOrder.LITTLE_ENDIAN)

    // GPS C-Struct: int64 (timestamp) + 3x float64 (lat,lon,alt) + 2x float32 (speed,acc) = 40 Bytes
    private val gpsByteArray = ByteArray(48)
    private val gpsBuffer = ByteBuffer.wrap(gpsByteArray).order(ByteOrder.LITTLE_ENDIAN)

    @SuppressLint("MissingPermission")
    fun startSensors(enableImu: Boolean, imuMode: String, enableGps: Boolean) {
        backgroundThread = HandlerThread("TelemetryBackground").also { it.start() }
        backgroundHandler = Handler(backgroundThread!!.looper)

        if (enableImu) {
            isRawMode = (imuMode == "Raw")
            if (isRawMode) {
                val accel = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
                val gyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
                // SENSOR_DELAY_FASTEST commands the hardware to push data as fast as physically possible (~200Hz)
                sensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_FASTEST, backgroundHandler)
                sensorManager.registerListener(this, gyro, SensorManager.SENSOR_DELAY_FASTEST, backgroundHandler)
                Log.d("TelemetryManager", "Started IMU in RAW mode")
            } else {
                // Game Rotation Vector = Fused Quaternion (No Magnetometer/EMI)
                val gameRot = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
                sensorManager.registerListener(this, gameRot, SensorManager.SENSOR_DELAY_FASTEST, backgroundHandler)
                Log.d("TelemetryManager", "Started IMU in FUSED mode")
            }
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
        val timestamp = System.nanoTime()

        if (isRawMode) {
            when (event.sensor.type) {
                Sensor.TYPE_LINEAR_ACCELERATION -> {
                    latestAccel[0] = event.values[0]
                    latestAccel[1] = event.values[1]
                    latestAccel[2] = event.values[2]
                    return // Do not send packet yet. Wait for Gyro to bundle them perfectly.
                }
                Sensor.TYPE_GYROSCOPE -> {
                    latestGyro[0] = event.values[0]
                    latestGyro[1] = event.values[1]
                    latestGyro[2] = event.values[2]

                    imuBuffer.clear()
                    imuBuffer.putLong(timestamp)
                    imuBuffer.put(1.toByte()) // Mode 1 = Raw
                    imuBuffer.putFloat(latestAccel[0]).putFloat(latestAccel[1]).putFloat(latestAccel[2])
                    imuBuffer.putFloat(latestGyro[0]).putFloat(latestGyro[1]).putFloat(latestGyro[2])
                    onImuData(imuByteArray)
                }
            }
        } else {
            if (event.sensor.type == Sensor.TYPE_GAME_ROTATION_VECTOR) {
                imuBuffer.clear()
                imuBuffer.putLong(timestamp)
                imuBuffer.put(0.toByte()) // Mode 0 = Fused Quaternion

                // Android Quaternion mapping: x, y, z, w (scalar is the 4th element)
                val x = event.values[0]
                val y = event.values[1]
                val z = event.values[2]
                val w = if (event.values.size > 3) event.values[3] else 1.0f

                imuBuffer.putFloat(x).putFloat(y).putFloat(z).putFloat(w)
                // Pad with zeros to keep the payload size identical to Raw mode (33 bytes)
                imuBuffer.putFloat(0.0f).putFloat(0.0f)
                onImuData(imuByteArray)
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    // --- GPS LISTENER ---
    override fun onLocationChanged(location: Location) {
        gpsBuffer.clear()
        gpsBuffer.putLong(System.nanoTime())
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
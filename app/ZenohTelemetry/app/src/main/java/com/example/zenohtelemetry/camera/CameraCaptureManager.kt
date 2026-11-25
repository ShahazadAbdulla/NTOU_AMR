package com.example.zenohtelemetry.camera

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.ImageFormat
import android.hardware.camera2.*
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Range
import android.view.Surface

enum class LensSelection {
    ULTRAWIDE,
    MAIN_WIDE,
    AUTO
}

class CameraCaptureManager(
    private val context: Context,
    private val onFrameCaptured: (ByteArray) -> Unit
) {
    private val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    private var cameraDevice: CameraDevice? = null
    private var captureSession: CameraCaptureSession? = null
    private var imageReader: ImageReader? = null

    private var backgroundThread: HandlerThread? = null
    private var backgroundHandler: Handler? = null

    private val targetWidth = 1280
    private val targetHeight = 720
    private var activeCameraId: String = "0"

    fun findCameraId(selection: LensSelection = LensSelection.MAIN_WIDE): String? {
        val backCameras = mutableListOf<Pair<String, Float>>()

        for (id in cameraManager.cameraIdList) {
            val characteristics = cameraManager.getCameraCharacteristics(id)
            val facing = characteristics.get(CameraCharacteristics.LENS_FACING)

            if (facing == CameraCharacteristics.LENS_FACING_BACK) {
                val focalLengths = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
                val minFocalLength = focalLengths?.minOrNull() ?: 4.0f
                backCameras.add(Pair(id, minFocalLength))
            }
        }

        if (backCameras.isEmpty()) {
            Log.e("CameraCapture", "No back-facing camera found!")
            return null
        }

        backCameras.sortBy { it.second }

        return when (selection) {
            LensSelection.ULTRAWIDE -> backCameras.first().first
            LensSelection.MAIN_WIDE -> {
                val index = if (backCameras.size > 1) 1 else 0
                backCameras[index].first
            }
            LensSelection.AUTO -> backCameras.first().first
        }
    }

    @SuppressLint("MissingPermission")
    fun startCamera(lensSelection: LensSelection = LensSelection.MAIN_WIDE) {
        startBackgroundThread()

        val cameraId = findCameraId(lensSelection) ?: return
        activeCameraId = cameraId

        imageReader = ImageReader.newInstance(targetWidth, targetHeight, ImageFormat.JPEG, 2).apply {
            setOnImageAvailableListener({ reader ->
                val image = reader.acquireLatestImage() ?: return@setOnImageAvailableListener
                try {
                    val plane = image.planes[0]
                    val buffer = plane.buffer
                    val bytes = ByteArray(buffer.remaining())
                    buffer.get(bytes)
                    onFrameCaptured(bytes)
                } catch (e: Exception) {
                    Log.e("CameraCapture", "Error extracting JPEG frame: ${e.message}")
                } finally {
                    image.close()
                }
            }, backgroundHandler)
        }

        cameraManager.openCamera(cameraId, object : CameraDevice.StateCallback() {
            override fun onOpened(camera: CameraDevice) {
                cameraDevice = camera
                createCaptureSession()
            }

            override fun onDisconnected(camera: CameraDevice) {
                camera.close()
                cameraDevice = null
            }

            override fun onError(camera: CameraDevice, error: Int) {
                camera.close()
                cameraDevice = null
                Log.e("CameraCapture", "CameraDevice error: $error")
            }
        }, backgroundHandler)
    }

    private fun createCaptureSession() {
        val device = cameraDevice ?: return
        val reader = imageReader ?: return

        val surfaces = listOf(reader.surface)

        device.createCaptureSession(surfaces, object : CameraCaptureSession.StateCallback() {
            override fun onConfigured(session: CameraCaptureSession) {
                captureSession = session
                try {
                    val characteristics = cameraManager.getCameraCharacteristics(activeCameraId)
                    val availableFpsRanges = characteristics.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)

                    // Find hardware Range(30, 30) or closest fixed 30fps boundary
                    val locked30FpsRange = availableFpsRanges?.firstOrNull { it.lower == 30 && it.upper == 30 }
                        ?: availableFpsRanges?.filter { it.upper == 30 }?.maxByOrNull { it.lower }
                        ?: Range(30, 30)

                    Log.d("CameraCapture", "Locking AE Target FPS Range to: $locked30FpsRange")

                    val requestBuilder = device.createCaptureRequest(CameraDevice.TEMPLATE_RECORD).apply {
                        addTarget(reader.surface)
                        set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO)
                        set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO)
                        set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON)
                        set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, locked30FpsRange)
                        set(CaptureRequest.JPEG_QUALITY, 70.toByte())
                    }

                    session.setRepeatingRequest(requestBuilder.build(), null, backgroundHandler)
                    Log.d("CameraCapture", "Strict 30 FPS Camera Session Active on Camera ID $activeCameraId")
                } catch (e: Exception) {
                    Log.e("CameraCapture", "Failed to start repeating capture request: ${e.message}")
                }
            }

            override fun onConfigureFailed(session: CameraCaptureSession) {
                Log.e("CameraCapture", "Failed to configure CameraCaptureSession")
            }
        }, backgroundHandler)
    }

    fun stopCamera() {
        try {
            captureSession?.stopRepeating()
            captureSession?.close()
            captureSession = null

            cameraDevice?.close()
            cameraDevice = null

            imageReader?.close()
            imageReader = null
        } catch (e: Exception) {
            Log.e("CameraCapture", "Error closing camera: ${e.message}")
        } finally {
            stopBackgroundThread()
        }
    }

    private fun startBackgroundThread() {
        backgroundThread = HandlerThread("CameraBackground").also { it.start() }
        backgroundHandler = Handler(backgroundThread!!.looper)
    }

    private fun stopBackgroundThread() {
        backgroundThread?.quitSafely()
        try {
            backgroundThread?.join()
            backgroundThread = null
            backgroundHandler = null
        } catch (e: InterruptedException) {
            Log.e("CameraCapture", "Error stopping camera thread: ${e.message}")
        }
    }
}
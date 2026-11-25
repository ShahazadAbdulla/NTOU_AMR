package com.example.zenohtelemetry.camera

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.ImageFormat
import android.hardware.camera2.*
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Size
import kotlin.math.abs

enum class LensSelection(val label: String) {
    MAIN_WIDE("Main (1x)"),
    ULTRAWIDE("Ultrawide (0.6x)"),
    SELFIE("Selfie (Front)")
}

class CameraCaptureManager(
    private val context: Context,
    private val onFrameCaptured: (jpegBytes: ByteArray, timestampNs: Long) -> Unit
) {
    private val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    private var cameraDevice: CameraDevice? = null
    private var captureSession: CameraCaptureSession? = null
    private var imageReader: ImageReader? = null

    private var backgroundThread: HandlerThread? = null
    private var backgroundHandler: Handler? = null

    private var activeLensSelection: LensSelection = LensSelection.MAIN_WIDE
    private var activeCameraId: String = "0"
    private var activeZoomRatio: Float = 1.0f

    fun getAvailableLenses(): List<LensSelection> {
        val available = mutableListOf<LensSelection>()
        var backId: String? = null
        var frontId: String? = null
        var backCount = 0

        for (id in cameraManager.cameraIdList) {
            val characteristics = cameraManager.getCameraCharacteristics(id)
            val facing = characteristics.get(CameraCharacteristics.LENS_FACING)
            if (facing == CameraCharacteristics.LENS_FACING_BACK) {
                if (backId == null) backId = id
                backCount++
            } else if (facing == CameraCharacteristics.LENS_FACING_FRONT) {
                if (frontId == null) frontId = id
            }
        }

        if (backId != null) {
            available.add(LensSelection.MAIN_WIDE)
            val chars = cameraManager.getCameraCharacteristics(backId)
            val zoomRange = chars.get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE)
            if ((zoomRange != null && zoomRange.lower < 0.95f) || backCount > 1) {
                available.add(LensSelection.ULTRAWIDE)
            }
        }

        if (frontId != null) {
            available.add(LensSelection.SELFIE)
        }

        return if (available.isEmpty()) listOf(LensSelection.MAIN_WIDE) else available
    }

    private fun selectCameraAndZoom(selection: LensSelection): Pair<String, Float> {
        val backIds = mutableListOf<Pair<String, Float>>()
        var frontId: String? = null

        for (id in cameraManager.cameraIdList) {
            val characteristics = cameraManager.getCameraCharacteristics(id)
            val facing = characteristics.get(CameraCharacteristics.LENS_FACING)
            if (facing == CameraCharacteristics.LENS_FACING_BACK) {
                val focalLengths = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
                val minFocal = focalLengths?.minOrNull() ?: 4.0f
                backIds.add(Pair(id, minFocal))
            } else if (facing == CameraCharacteristics.LENS_FACING_FRONT && frontId == null) {
                frontId = id
            }
        }
        backIds.sortBy { it.second }

        val primaryBackId = backIds.firstOrNull()?.first ?: cameraManager.cameraIdList.firstOrNull() ?: "0"

        when (selection) {
            LensSelection.SELFIE -> {
                val camId = frontId ?: primaryBackId
                return Pair(camId, 1.0f)
            }
            LensSelection.ULTRAWIDE -> {
                val chars = cameraManager.getCameraCharacteristics(primaryBackId)
                val zoomRange = chars.get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE)
                if (zoomRange != null && zoomRange.lower < 0.95f) {
                    // Modern logical camera zoom switching (e.g. 0.5x / 0.6x on Primary Back ID)
                    return Pair(primaryBackId, zoomRange.lower)
                }
                // Fallback for older multi-camera devices with separate physical IDs
                if (backIds.size > 1) {
                    return Pair(backIds.first().first, 1.0f)
                }
                return Pair(primaryBackId, 1.0f)
            }
            LensSelection.MAIN_WIDE -> {
                val chars = cameraManager.getCameraCharacteristics(primaryBackId)
                val zoomRange = chars.get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE)
                if (zoomRange != null && zoomRange.lower < 0.95f && zoomRange.upper >= 1.0f) {
                    return Pair(primaryBackId, 1.0f)
                }
                val mainId = if (backIds.size > 1) backIds[1].first else primaryBackId
                return Pair(mainId, 1.0f)
            }
        }
    }

    @SuppressLint("MissingPermission")
    fun startCamera(lensSelection: LensSelection = LensSelection.MAIN_WIDE) {
        startBackgroundThread()

        activeLensSelection = lensSelection
        val (cameraId, zoomRatio) = selectCameraAndZoom(lensSelection)
        activeCameraId = cameraId
        activeZoomRatio = zoomRatio

        val characteristics = cameraManager.getCameraCharacteristics(activeCameraId)
        val map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
        val jpegSizes = map?.getOutputSizes(ImageFormat.JPEG)

        val chosenSize = jpegSizes?.firstOrNull { it.width == 1280 && it.height == 720 }
            ?: jpegSizes?.minByOrNull { abs((it.width * it.height) - (1280 * 720)) }
            ?: Size(1280, 720)

        Log.d("CameraCapture", "Selected camera ID: $activeCameraId with zoom: $activeZoomRatio and size: $chosenSize")

        imageReader = ImageReader.newInstance(chosenSize.width, chosenSize.height, ImageFormat.JPEG, 2).apply {
            setOnImageAvailableListener({ reader ->
                val image = reader.acquireLatestImage() ?: return@setOnImageAvailableListener
                try {
                    val timestampNs = image.timestamp
                    val plane = image.planes[0]
                    val buffer = plane.buffer
                    val bytes = ByteArray(buffer.remaining())
                    buffer.get(bytes)
                    onFrameCaptured(bytes, timestampNs)
                } catch (e: Exception) {
                    Log.e("CameraCapture", "Error extracting JPEG frame: ${e.message}")
                } finally {
                    image.close()
                }
            }, backgroundHandler)
        }

        cameraManager.openCamera(activeCameraId, object : CameraDevice.StateCallback() {
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
                Log.e("CameraCapture", "CameraDevice error: $error on camera ID $activeCameraId")
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

                    // 1. Safe AF Mode selection
                    val availableAfModes = characteristics.get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES) ?: intArrayOf()
                    val afMode = when {
                        availableAfModes.contains(CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO) ->
                            CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO
                        availableAfModes.contains(CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE) ->
                            CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE
                        availableAfModes.contains(CaptureRequest.CONTROL_AF_MODE_AUTO) ->
                            CaptureRequest.CONTROL_AF_MODE_AUTO
                        else ->
                            CaptureRequest.CONTROL_AF_MODE_OFF
                    }

                    // 2. Safe 30 FPS AE Range selection
                    val availableFpsRanges = characteristics.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)
                    val fpsRange = availableFpsRanges?.firstOrNull { it.lower == 30 && it.upper == 30 }
                        ?: availableFpsRanges?.filter { it.upper == 30 }?.maxByOrNull { it.lower }
                        ?: availableFpsRanges?.lastOrNull()

                    Log.d("CameraCapture", "Configuring session: AF=$afMode, FPS=$fpsRange, Zoom=$activeZoomRatio")

                    // TEMPLATE_PREVIEW is far more reliable on Samsung Front/Aux cameras with a single JPEG surface
                    val requestBuilder = device.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply {
                        addTarget(reader.surface)
                        set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO)
                        set(CaptureRequest.CONTROL_AF_MODE, afMode)
                        set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON)
                        if (fpsRange != null) {
                            set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fpsRange)
                        }

                        // Apply 0.6x Ultrawide or 1.0x Main zoom ratio if supported
                        val zoomRange = characteristics.get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE)
                        if (zoomRange != null) {
                            val clampedZoom = activeZoomRatio.coerceIn(zoomRange.lower, zoomRange.upper)
                            set(CaptureRequest.CONTROL_ZOOM_RATIO, clampedZoom)
                        }

                        set(CaptureRequest.JPEG_QUALITY, 70.toByte())
                    }

                    session.setRepeatingRequest(requestBuilder.build(), null, backgroundHandler)
                    Log.d("CameraCapture", "Camera Session Active on Camera ID $activeCameraId")
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
            imageReader?.setOnImageAvailableListener(null, null)

            captureSession?.stopRepeating()
            captureSession?.close()
            captureSession = null

            cameraDevice?.close()
            cameraDevice = null

            imageReader?.close()
            imageReader = null

            Thread.sleep(50)
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

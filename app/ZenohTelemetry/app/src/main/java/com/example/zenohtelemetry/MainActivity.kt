package com.example.zenohtelemetry

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.animation.AnimatedVisibility
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import com.example.zenohtelemetry.camera.CameraCaptureManager
import com.example.zenohtelemetry.camera.LensSelection
import com.example.zenohtelemetry.sensors.TelemetryManager
import com.example.zenohtelemetry.transport.ZenohTransport
import io.zenoh.Zenoh
import io.zenoh.qos.Reliability
import kotlinx.coroutines.delay
import kotlin.concurrent.thread

data class LogEntry(val text: String, val isError: Boolean = false, val isSuccess: Boolean = false)

class MainActivity : ComponentActivity() {
    private lateinit var cameraManager: CameraCaptureManager
    private lateinit var telemetryManager: TelemetryManager
    private val zenohTransport = ZenohTransport()
    private var frameSequence = 0L

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Zenoh.initLogFromEnvOr("error")

        cameraManager = CameraCaptureManager(this) { jpegBytes ->
            zenohTransport.publishCameraFrame(jpegBytes, frameSequence++)
        }

        telemetryManager = TelemetryManager(
            this,
            onImuData = { bytes -> zenohTransport.publishImuData(bytes) },
            onGpsData = { bytes -> zenohTransport.publishGpsData(bytes) }
        )

        setContent {
            MaterialTheme(
                colorScheme = darkColorScheme(
                    background = Color.Black,
                    surface = Color(0xFF1E1E1E),
                    onBackground = Color.White,
                    onSurface = Color.White
                )
            ) {
                Surface(modifier = Modifier.fillMaxSize()) {
                    ZenoneDashboard()
                }
            }
        }
    }

    @Composable
    fun ZenoneDashboard() {
        var ipAddress by remember { mutableStateOf("172.28.51.207") }
        var showAdvanced by remember { mutableStateOf(false) }
        var showTerminal by remember { mutableStateOf(true) }

        // Networking State
        var mode by remember { mutableStateOf("Client") }
        var transport by remember { mutableStateOf("TCP Unicast") }
        var isConnected by remember { mutableStateOf(false) }

        // Streaming State & Sensor Toggles
        var isStreaming by remember { mutableStateOf(false) }
        var enableCamera by remember { mutableStateOf(true) }
        var enableImu by remember { mutableStateOf(true) }
        var enableGps by remember { mutableStateOf(false) }
        var imuMode by remember { mutableStateOf("Fused") }

        val logs = remember { mutableStateListOf<LogEntry>() }
        val listState = rememberLazyListState()

        val context = LocalContext.current
        var hasCameraPermission by remember { mutableStateOf(ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) }
        var hasLocationPermission by remember { mutableStateOf(ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) }

        LaunchedEffect(logs.size) {
            if (logs.isNotEmpty()) listState.animateScrollToItem(logs.size - 1)
        }

        fun appendLog(msg: String, isError: Boolean = false, isSuccess: Boolean = false) {
            runOnUiThread { logs.add(LogEntry(msg, isError, isSuccess)) }
        }

        // 1Hz Telemetry Logger
        LaunchedEffect(isStreaming) {
            if (isStreaming) {
                while (true) {
                    delay(1000)
                    val camStat = if (enableCamera) "30Hz" else "OFF"
                    val imuStat = if (enableImu) "200Hz ($imuMode)" else "OFF"
                    val gpsStat = if (enableGps) "15Hz" else "OFF"
                    appendLog("[STREAMING] Transport: ${transport.split(" ")[0]} | Camera: $camStat | IMU: $imuStat | GPS: $gpsStat")
                }
            }
        }

        val permissionLauncher = rememberLauncherForActivityResult(
            contract = ActivityResultContracts.RequestMultiplePermissions()
        ) { perms ->
            hasCameraPermission = perms[Manifest.permission.CAMERA] ?: hasCameraPermission
            hasLocationPermission = perms[Manifest.permission.ACCESS_FINE_LOCATION] ?: hasLocationPermission

            if (hasCameraPermission && (!enableGps || hasLocationPermission)) {
                appendLog("SUCCESS: Required permissions granted.", false, true)
            } else {
                appendLog("FAILED: Missing hardware permissions.", true, false)
            }
        }

        Column(
            modifier = Modifier.fillMaxSize().padding(24.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Spacer(modifier = Modifier.height(16.dp))

            // Header with Connection Status
            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                Text("PROJECT ZERO", style = MaterialTheme.typography.headlineMedium, fontWeight = FontWeight.Bold, letterSpacing = 2.sp)
                Spacer(modifier = Modifier.height(4.dp))
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Box(modifier = Modifier.size(10.dp).background(if (isConnected) Color(0xFF55FF55) else Color(0xFFFF5555), shape = CircleShape))
                    Spacer(modifier = Modifier.width(6.dp))
                    Text(if (isConnected) "CONNECTED" else "DISCONNECTED", color = if (isConnected) Color(0xFF55FF55) else Color(0xFFFF5555), fontSize = 12.sp, fontWeight = FontWeight.Bold)
                }
            }

            Spacer(modifier = Modifier.height(24.dp))

            // STAGE 1: Network Config
            AnimatedVisibility(visible = !isStreaming) {
                Card(
                    shape = RoundedCornerShape(16.dp),
                    colors = CardDefaults.cardColors(containerColor = Color(0xFF1E1E1E)),
                    modifier = Modifier.fillMaxWidth()
                ) {
                    Column(modifier = Modifier.padding(16.dp), horizontalAlignment = Alignment.CenterHorizontally) {
                        OutlinedTextField(
                            value = ipAddress,
                            onValueChange = { ipAddress = it },
                            label = { Text("Target Host IP") },
                            shape = RoundedCornerShape(12.dp),
                            modifier = Modifier.fillMaxWidth(),
                            enabled = !isConnected,
                            colors = OutlinedTextFieldDefaults.colors(focusedBorderColor = Color.White, unfocusedBorderColor = Color.Gray)
                        )
                    }
                }
            }

            Spacer(modifier = Modifier.height(8.dp))

            if (!isStreaming) {
                TextButton(onClick = { showAdvanced = !showAdvanced }) {
                    Text(if (showAdvanced) "Hide Advanced Options" else "Show Advanced Options", color = Color.Gray)
                }

                AnimatedVisibility(visible = showAdvanced) {
                    Card(
                        shape = RoundedCornerShape(16.dp),
                        colors = CardDefaults.cardColors(containerColor = Color(0xFF121212)),
                        modifier = Modifier.fillMaxWidth().padding(vertical = 4.dp)
                    ) {
                        Column(modifier = Modifier.padding(16.dp)) {
                            DropdownSelector("Mode", listOf("Client", "Peer"), mode, !isConnected) { mode = it }
                            Spacer(modifier = Modifier.height(8.dp))
                            DropdownSelector("Transport", listOf("TCP Unicast", "UDP Unicast"), transport, !isConnected) { transport = it }
                            Spacer(modifier = Modifier.height(8.dp))
                            Row(verticalAlignment = Alignment.CenterVertically) {
                                Checkbox(checked = showTerminal, onCheckedChange = { showTerminal = it }, colors = CheckboxDefaults.colors(checkedColor = Color.White, checkmarkColor = Color.Black))
                                Text("Show Terminal Logs", fontSize = 14.sp, color = Color.LightGray)
                            }
                        }
                    }
                }
            }

            // STAGE 2: Sensor Selection
            AnimatedVisibility(visible = isConnected && !isStreaming) {
                Card(
                    shape = RoundedCornerShape(16.dp),
                    colors = CardDefaults.cardColors(containerColor = Color(0xFF2A2A2A)),
                    modifier = Modifier.fillMaxWidth().padding(vertical = 12.dp)
                ) {
                    Column(modifier = Modifier.padding(16.dp)) {
                        Text("SENSOR ROUTING", fontWeight = FontWeight.Bold, color = Color.White, fontSize = 14.sp, modifier = Modifier.padding(bottom = 8.dp))

                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Checkbox(checked = enableCamera, onCheckedChange = { enableCamera = it }, colors = CheckboxDefaults.colors(checkedColor = Color.White, checkmarkColor = Color.Black))
                            Text("Camera Stream", color = Color.White)
                        }

                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Checkbox(checked = enableImu, onCheckedChange = { enableImu = it }, colors = CheckboxDefaults.colors(checkedColor = Color.White, checkmarkColor = Color.Black))
                            Text("IMU Data (200Hz)", color = Color.White)
                        }

                        AnimatedVisibility(visible = enableImu) {
                            Column(modifier = Modifier.padding(start = 32.dp)) {
                                Row(verticalAlignment = Alignment.CenterVertically) {
                                    RadioButton(selected = imuMode == "Fused", onClick = { imuMode = "Fused" }, colors = RadioButtonDefaults.colors(selectedColor = Color.White))
                                    Text("Fused (Quaternion)", color = Color.LightGray, fontSize = 14.sp)
                                }
                                Row(verticalAlignment = Alignment.CenterVertically) {
                                    RadioButton(selected = imuMode == "Raw", onClick = { imuMode = "Raw" }, colors = RadioButtonDefaults.colors(selectedColor = Color.White))
                                    Text("Raw (Accel/Gyro)", color = Color.LightGray, fontSize = 14.sp)
                                }
                            }
                        }

                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Checkbox(checked = enableGps, onCheckedChange = { enableGps = it }, colors = CheckboxDefaults.colors(checkedColor = Color.White, checkmarkColor = Color.Black))
                            Text("GPS Telemetry (15Hz)", color = Color.White)
                        }
                    }
                }
            }

            Spacer(modifier = Modifier.height(16.dp))

            // Action Buttons
            if (!isConnected) {
                Button(
                    onClick = {
                        val endpoint = when (transport) {
                            "TCP Unicast" -> "tcp/$ipAddress:7447"
                            else -> "udp/$ipAddress:7447"
                        }
                        appendLog("Connecting to $endpoint...", false, false)
                        thread {
                            // Temporary hardcode for best_effort until ZenohTransport is updated
                            val result = zenohTransport.connect(endpoint, mode.lowercase())
                            if (zenohTransport.isConnected) {
                                isConnected = true
                                appendLog(result, isError = false, isSuccess = true)
                            } else {
                                appendLog(result, isError = true, isSuccess = false)
                            }
                        }
                    },
                    modifier = Modifier.fillMaxWidth().height(56.dp),
                    shape = RoundedCornerShape(28.dp),
                    colors = ButtonDefaults.buttonColors(containerColor = Color.White, contentColor = Color.Black)
                ) {
                    Text("CONNECT HOST", fontWeight = FontWeight.Bold)
                }
            } else if (!isStreaming) {
                Column(modifier = Modifier.fillMaxWidth(), horizontalAlignment = Alignment.CenterHorizontally) {
                    Button(
                        onClick = {
                            if (!hasCameraPermission || (enableGps && !hasLocationPermission)) {
                                val perms = mutableListOf(Manifest.permission.CAMERA)
                                if (enableGps) perms.add(Manifest.permission.ACCESS_FINE_LOCATION)
                                appendLog("Requesting OS Hardware Permissions...", false, false)
                                permissionLauncher.launch(perms.toTypedArray())
                                return@Button
                            }

                            if (enableCamera) {
                                cameraManager.startCamera(LensSelection.MAIN_WIDE)
                                appendLog("Camera Pipeline Activated.", false, true)
                            }
                            if (enableImu || enableGps) {
                                telemetryManager.startSensors(enableImu, imuMode, enableGps)
                                appendLog("Hardware Sensors Activated.", false, true)
                            }
                            isStreaming = true
                        },
                        modifier = Modifier.fillMaxWidth().height(56.dp),
                        shape = RoundedCornerShape(28.dp),
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF55FF55), contentColor = Color.Black)
                    ) {
                        Text("START STREAMING", fontWeight = FontWeight.Bold)
                    }

                    Spacer(modifier = Modifier.height(8.dp))

                    TextButton(onClick = {
                        zenohTransport.disconnect()
                        isConnected = false
                        appendLog("Disconnected from Host.", false, false)
                    }) {
                        Text("Disconnect", color = Color.Gray)
                    }
                }
            } else {
                Button(
                    onClick = {
                        cameraManager.stopCamera()
                        telemetryManager.stopSensors()
                        zenohTransport.disconnect()
                        isStreaming = false
                        isConnected = false
                        appendLog("Hardware Halted. Disconnected from Host.", false, false)
                    },
                    modifier = Modifier.fillMaxWidth().height(56.dp),
                    shape = RoundedCornerShape(28.dp),
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFFF5555), contentColor = Color.White)
                ) {
                    Text("STOP STREAMING & DISCONNECT", fontWeight = FontWeight.Bold)
                }
            }

            Spacer(modifier = Modifier.height(24.dp))

            AnimatedVisibility(visible = showTerminal) {
                Box(
                    modifier = Modifier.fillMaxWidth().weight(1f).background(Color(0xFF0A0A0A), RoundedCornerShape(8.dp)).border(1.dp, Color(0xFF333333), RoundedCornerShape(8.dp)).padding(12.dp)
                ) {
                    LazyColumn(state = listState, modifier = Modifier.fillMaxSize()) {
                        items(logs) { log ->
                            val color = when {
                                log.isError -> Color(0xFFFF5555)
                                log.isSuccess -> Color(0xFF55FF55)
                                else -> Color(0xFFAAAAAA)
                            }
                            Text("> ${log.text}", color = color, fontFamily = FontFamily.Monospace, fontSize = 12.sp, modifier = Modifier.padding(vertical = 2.dp))
                        }
                    }
                }
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        cameraManager.stopCamera()
        telemetryManager.stopSensors()
        zenohTransport.disconnect()
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun DropdownSelector(label: String, options: List<String>, selected: String, enabled: Boolean = true, onSelected: (String) -> Unit) {
    var expanded by remember { mutableStateOf(false) }
    ExposedDropdownMenuBox(expanded = expanded, onExpandedChange = { if (enabled) expanded = !expanded }) {
        OutlinedTextField(
            value = selected,
            onValueChange = {},
            readOnly = true,
            label = { Text(label) },
            shape = RoundedCornerShape(12.dp),
            trailingIcon = { ExposedDropdownMenuDefaults.TrailingIcon(expanded = expanded) },
            modifier = Modifier.menuAnchor().fillMaxWidth(),
            enabled = enabled,
            colors = OutlinedTextFieldDefaults.colors(focusedBorderColor = Color.White, unfocusedBorderColor = Color.Gray, disabledBorderColor = Color.DarkGray, disabledTextColor = Color.Gray)
        )
        ExposedDropdownMenu(expanded = expanded, onDismissRequest = { expanded = false }) {
            options.forEach { opt ->
                DropdownMenuItem(text = { Text(opt) }, onClick = { onSelected(opt); expanded = false })
            }
        }
    }
}
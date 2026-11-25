package com.example.zenohtelemetry

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.net.wifi.WifiManager
import android.os.Build
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
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
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
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

data class LogEntry(val text: String, val isError: Boolean = false, val isSuccess: Boolean = false)

/**
 * Android 17 (API 37) Local Network Protection permission. Referenced as a literal so the app
 * still links with minSdk 30, where the Manifest.permission constant does not exist.
 */
private const val PERMISSION_ACCESS_LOCAL_NETWORK = "android.permission.ACCESS_LOCAL_NETWORK"

class MainActivity : ComponentActivity() {
    private lateinit var cameraManager: CameraCaptureManager
    private lateinit var telemetryManager: TelemetryManager
    private val zenohTransport = ZenohTransport()
    private var frameSequence = 0L

    private var wifiLock: WifiManager.WifiLock? = null

    private fun acquireLowLatencyWifiLock() {
        val wm = applicationContext.getSystemService(WIFI_SERVICE) as WifiManager
        if (wifiLock == null) {
            wifiLock = wm.createWifiLock(WifiManager.WIFI_MODE_FULL_LOW_LATENCY, "ProjectZeRo:LowLatency")
            wifiLock?.setReferenceCounted(false)
        }
        wifiLock?.acquire()
    }

    private fun releaseLowLatencyWifiLock() {
        runCatching { if (wifiLock?.isHeld == true) wifiLock?.release() }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        Zenoh.initLogFromEnvOr("error")

        cameraManager = CameraCaptureManager(this) { jpegBytes, timestampNs ->
            zenohTransport.publishCameraFrame(jpegBytes, frameSequence++, timestampNs)
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
        val context = LocalContext.current
        val prefs = remember { context.getSharedPreferences("project_zero_prefs", Context.MODE_PRIVATE) }
        var ipAddress by remember { mutableStateOf(prefs.getString("host_ip", "192.168.51.34") ?: "192.168.51.34") }
        var showAdvanced by remember { mutableStateOf(false) }
        var showTerminal by remember { mutableStateOf(true) }

        // Networking State
        var mode by remember { mutableStateOf("Peer") }
        var transport by remember { mutableStateOf("TCP Unicast") }
        var isConnected by remember { mutableStateOf(false) }

        // Streaming State & Sensor Toggles
        var isStreaming by remember { mutableStateOf(false) }
        var enableCamera by remember { mutableStateOf(true) }
        var enableImu by remember { mutableStateOf(true) }
        var enableGps by remember { mutableStateOf(false) }
        var imuRate by remember { mutableStateOf("200 Hz") }

        val availableLenses = remember { cameraManager.getAvailableLenses() }
        var selectedLens by remember { mutableStateOf(LensSelection.MAIN_WIDE) }

        val logs = remember { mutableStateListOf<LogEntry>() }
        val listState = rememberLazyListState()

        var hasCameraPermission by remember { mutableStateOf(ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) }
        var hasLocationPermission by remember { mutableStateOf(ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) }
        var hasLocalNetworkPermission by remember {
            mutableStateOf(
                Build.VERSION.SDK_INT < 37 ||
                ContextCompat.checkSelfPermission(context, PERMISSION_ACCESS_LOCAL_NETWORK) == PackageManager.PERMISSION_GRANTED
            )
        }
        val scope = rememberCoroutineScope()

        LaunchedEffect(logs.size) {
            if (logs.isNotEmpty()) listState.animateScrollToItem(logs.size - 1)
        }

        fun appendLog(msg: String, isError: Boolean = false, isSuccess: Boolean = false) {
            runOnUiThread {
                if (logs.size >= 100) logs.removeAt(0)
                logs.add(LogEntry(msg, isError, isSuccess))
            }
        }

        // 1Hz Telemetry Logger
        LaunchedEffect(isStreaming) {
            if (isStreaming) {
                while (true) {
                    delay(1000)
                    val stats = zenohTransport.getAndResetStats()
                    val camStat = if (enableCamera) "${stats.camSent} FPS (${stats.camDrops} drops)" else "OFF"
                    val imuStat = if (enableImu) "${stats.imuSent} Hz (${stats.imuDrops} drops)" else "OFF"
                    val gpsStat = if (enableGps) "${stats.gpsSent} Hz (${stats.gpsDrops} drops)" else "OFF"
                    appendLog("[STATS] Camera: $camStat | IMU: $imuStat | GPS: $gpsStat")
                }
            }
        }

        // 30s Background Clock Resync
        LaunchedEffect(isConnected) {
            if (isConnected) {
                while (true) {
                    delay(30_000L)
                    val syncMsg = withContext(Dispatchers.IO) {
                        zenohTransport.runClockSyncBurst()
                    }
                    appendLog("[SYNC] $syncMsg", isError = false, isSuccess = syncMsg.startsWith("CLOCK SYNCED"))
                }
            }
        }

        val permissionLauncher = rememberLauncherForActivityResult(
            contract = ActivityResultContracts.RequestMultiplePermissions()
        ) { perms ->
            // Android 17 LNP: ACCESS_LOCAL_NETWORK gates all LAN traffic.
            perms[PERMISSION_ACCESS_LOCAL_NETWORK]?.let { granted ->
                hasLocalNetworkPermission = granted
                if (granted) appendLog("Local Network access granted.", false, true)
                else appendLog("DENIED: Local Network access blocked (Android 17 LNP); host will be unreachable.", true, false)
            }
            if (perms.containsKey(Manifest.permission.CAMERA)) {
                hasCameraPermission = perms.getValue(Manifest.permission.CAMERA)
                hasLocationPermission = perms[Manifest.permission.ACCESS_FINE_LOCATION] ?: hasLocationPermission
                if (hasCameraPermission && (!enableGps || hasLocationPermission)) {
                    appendLog("SUCCESS: Required permissions granted.", false, true)
                } else {
                    appendLog("FAILED: Missing hardware permissions.", true, false)
                }
            }
        }

        val scrollState = rememberScrollState()

        Column(
            modifier = Modifier.fillMaxSize().verticalScroll(scrollState).padding(horizontal = 16.dp, vertical = 12.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Spacer(modifier = Modifier.height(8.dp))

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

            Spacer(modifier = Modifier.height(12.dp))

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
                            onValueChange = {
                                ipAddress = it
                                prefs.edit().putString("host_ip", it).apply()
                            },
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

                        AnimatedVisibility(visible = enableCamera && availableLenses.size > 1) {
                            Column(modifier = Modifier.padding(start = 32.dp)) {
                                availableLenses.forEach { lens ->
                                    Row(verticalAlignment = Alignment.CenterVertically) {
                                        RadioButton(
                                            selected = (selectedLens == lens),
                                            onClick = { selectedLens = lens },
                                            colors = RadioButtonDefaults.colors(selectedColor = Color.White)
                                        )
                                        Text(lens.label, color = Color.LightGray, fontSize = 14.sp)
                                    }
                                }
                            }
                        }

                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Checkbox(checked = enableImu, onCheckedChange = { enableImu = it }, colors = CheckboxDefaults.colors(checkedColor = Color.White, checkmarkColor = Color.Black))
                            Text("IMU Data (Universal 6-Axis + Quat)", color = Color.White)
                        }

                        AnimatedVisibility(visible = enableImu) {
                            Column(modifier = Modifier.padding(start = 32.dp, top = 4.dp, bottom = 8.dp)) {
                                DropdownSelector(
                                    label = "IMU Rate",
                                    options = listOf("50 Hz", "100 Hz", "200 Hz", "400 Hz"),
                                    selected = imuRate,
                                    enabled = !isStreaming
                                ) { imuRate = it }
                            }
                        }

                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Checkbox(checked = enableGps, onCheckedChange = { enableGps = it }, colors = CheckboxDefaults.colors(checkedColor = Color.White, checkmarkColor = Color.Black))
                            Text("GPS Telemetry (1Hz)", color = Color.White)
                        }
                    }
                }
            }

            Spacer(modifier = Modifier.height(16.dp))

            // Action Buttons
            if (!isConnected) {
                Button(
                    onClick = {
                        if (!hasLocalNetworkPermission) {
                            appendLog("Requesting Local Network permission (Android 17 LNP required for LAN)...", false, false)
                            permissionLauncher.launch(arrayOf(PERMISSION_ACCESS_LOCAL_NETWORK))
                            return@Button
                        }
                        val endpoint = when (transport) {
                            "TCP Unicast" -> "tcp/$ipAddress:7447"
                            else -> "udp/$ipAddress:7447"
                        }
                        appendLog("Connecting to $endpoint...", false, false)
                        scope.launch {
                            val result = withContext(Dispatchers.IO) {
                                val openResult = zenohTransport.connect(endpoint, mode.lowercase())
                                // Zenoh.open() is asynchronous: it returns before the handshake completes,
                                // so poll for a real router/peer link before declaring success.
                                if (zenohTransport.isConnected && !zenohTransport.awaitActiveLink(3_000)) {
                                    "FAILED: No link to $endpoint within 3s. Confirm the host router is listening and both devices share the LAN."
                                } else openResult
                            }
                            if (zenohTransport.isConnected && zenohTransport.hasActiveLink()) {
                                isConnected = true
                                appendLog(result, isError = false, isSuccess = true)
                                scope.launch(Dispatchers.IO) {
                                    val syncMsg = zenohTransport.runClockSyncBurst()
                                    appendLog(syncMsg, isError = false, isSuccess = syncMsg.startsWith("CLOCK SYNCED"))
                                }
                            } else {
                                zenohTransport.disconnect()
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
                                cameraManager.startCamera(selectedLens)
                                appendLog("Camera Pipeline Activated (${selectedLens.label}).", false, true)
                            }
                            if (enableImu || enableGps) {
                                telemetryManager.startSensors(enableImu, imuRate, enableGps)
                                appendLog("Hardware Sensors Activated ($imuRate IMU).", false, true)
                            }
                            acquireLowLatencyWifiLock()
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
                        releaseLowLatencyWifiLock()
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
                        releaseLowLatencyWifiLock()
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

            Spacer(modifier = Modifier.height(12.dp))

            AnimatedVisibility(visible = showTerminal) {
                Box(
                    modifier = Modifier.fillMaxWidth().height(180.dp).background(Color(0xFF0A0A0A), RoundedCornerShape(8.dp)).border(1.dp, Color(0xFF333333), RoundedCornerShape(8.dp)).padding(12.dp)
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
        releaseLowLatencyWifiLock()
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
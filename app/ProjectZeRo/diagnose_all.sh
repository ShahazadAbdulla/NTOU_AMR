#!/usr/bin/env bash
echo "================================================================="
echo " PROJECT ZERO — FULL NATIVE CRASH, APK & PIPELINE DIAGNOSTIC"
echo "================================================================="

echo -e "\n=== 1. ANDROID NATIVE & JAVA CRASH BUFFER (logcat -b crash) ==="
adb logcat -b crash -d -t 200

echo -e "\n=== 2. FATAL / SIGSEGV / TOMBSTONE / JNI ERRORS IN MAIN LOGCAT ==="
adb logcat -d -t 2000 | grep -i -E "FATAL|SIGSEGV|SIGABRT|backtrace|libzenoh|AndroidRuntime|ZenohTransport|CameraCapture|TelemetryManager| IllegalStateException|NullPointerException" | tail -n 120

echo -e "\n=== 3. CURRENT APP PROCESS & PERMISSIONS STATE ==="
echo -n "App PID: "
adb shell pidof com.example.zenohtelemetry || echo "(NOT RUNNING / CRASHED)"
adb shell dumpsys package com.example.zenohtelemetry | grep -A 15 "requested permissions:"
adb shell appops get com.example.zenohtelemetry ACCESS_LOCAL_NETWORK

echo -e "\n=== 4. PHONE & HOST NETWORK TOPOLOGY ==="
echo "--- Host IPs ---"
ip -4 -brief addr
echo "--- Host Port 7447 Sockets ---"
ss -tanp | grep 7447 || echo "(No TCP 7447 sockets)"
ss -uanp | grep 7447 || echo "(No UDP 7447 sockets)"
echo "--- Phone IPs & Routes ---"
adb shell ip -4 -brief addr
adb shell ip route

echo -e "\n=== 5. LIVE CRASH REPRODUCTION CAPTURE ==="
echo "Clearing logcat, launching app, waiting 6 seconds..."
adb logcat -c
adb shell am force-stop com.example.zenohtelemetry
sleep 1
adb shell am start -n com.example.zenohtelemetry/.MainActivity
sleep 4
echo "App PID after launch: $(adb shell pidof com.example.zenohtelemetry)"
echo "--- Logs during launch ---"
adb logcat -d | grep -i -E "FATAL|AndroidRuntime|libzenoh|ZenohTransport|CameraCapture|TelemetryManager|zenohtelemetry" | tail -n 80

echo "================================================================="
echo " DIAGNOSTIC COMPLETE"
echo "================================================================="

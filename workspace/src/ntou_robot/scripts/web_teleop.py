#!/usr/bin/env python3
"""NTOU STM32 web controller. Standard library only; no ROS, Docker, Flask or pyserial."""

from __future__ import annotations

import argparse
import hmac
import json
import os
import secrets
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

MAX_LINEAR_X = 0.5   # matches max_linear_x_mps in hardware_interface
MAX_ANGULAR_Z = 2.0  # matches max_angular_z_rad_s in hardware_interface
MAX_STEER_DEG = 35   # matches max_steering_degrees in hardware_interface

WHEELS = ("FL", "FR", "RR", "RL")
STEERING = ("SR", "SL")
DRIVE_START_PERCENT = 20
DRIVE_STEP_PERCENT = 10
DRIVE_MAX_PERCENT = 50
STEERING_STEPS = (10, 20, 30, 35)
HEARTBEAT_TIMEOUT_S = 1.25


PAGE = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="theme-color" content="#0a1020">
<title>NTOU Robot Controller</title>
<style>
:root{color-scheme:dark;--bg:#09101e;--panel:#121d31;--panel2:#18263d;--line:#2b3b56;--text:#edf4ff;--muted:#9db0c9;--blue:#3278d4;--green:#20a46b;--red:#d83b4c;--amber:#d5a23e}
*{box-sizing:border-box;touch-action:manipulation}
html,body{margin:0;width:100%;height:100%;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;overflow:hidden}
body{padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}
.app{height:100%;min-height:0;display:flex;flex-direction:column;gap:10px;padding:12px 14px}
.top{display:flex;align-items:center;justify-content:space-between;gap:12px;min-height:42px}
.brand{font-weight:800;letter-spacing:.08em;font-size:clamp(14px,2vw,19px)}
.statusline{display:flex;gap:8px;align-items:center;flex-wrap:wrap;justify-content:flex-end;color:var(--muted);font-size:12px}
.badge{border:1px solid var(--line);border-radius:999px;padding:5px 10px;background:#111b2d}
.badge.ok{color:#79e0ae;border-color:#256847}.badge.bad{color:#ff9aa4;border-color:#873744}.badge.warn{color:#ffd27c;border-color:#755b28}
.layout{display:grid;grid-template-columns:minmax(0,1fr) minmax(150px,.72fr) minmax(0,1fr);gap:12px;flex:1;min-height:0}
.panel{background:linear-gradient(155deg,var(--panel),#0f192b);border:1px solid var(--line);border-radius:20px;padding:clamp(10px,2vw,20px);display:flex;flex-direction:column;min-height:0;box-shadow:0 10px 30px #0003}
.panelhead{text-align:center;color:var(--muted);font-weight:700;text-transform:uppercase;letter-spacing:.12em;font-size:12px;margin:2px 0 8px}
.bigvalue{text-align:center;font-size:clamp(22px,4vw,38px);font-weight:800;line-height:1.1;margin:3px 0 8px;font-variant-numeric:tabular-nums}
.subvalue{text-align:center;color:var(--muted);font-size:12px;min-height:18px;margin-bottom:10px}
.pad{flex:1;display:grid;grid-template-rows:1fr 1fr;gap:12px;min-height:0}
.pad.steer{grid-template-columns:1fr 1fr;grid-template-rows:1fr}
.action{border:1px solid #39516f;border-radius:18px;background:linear-gradient(160deg,#203653,#15243a);color:var(--text);font-size:clamp(16px,2.5vw,24px);font-weight:800;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:6px;min-height:0;user-select:none;-webkit-user-select:none;cursor:pointer;box-shadow:inset 0 1px #ffffff12,0 5px 14px #0003;transition:transform .06s,filter .08s}
.action:active{transform:scale(.97);filter:brightness(1.25)}
.action .arrow{font-size:clamp(42px,8vw,84px);line-height:.9}
.action small{font-size:11px;letter-spacing:.12em;color:#b9cbe2}
.action.forward{border-color:#256b55;background:linear-gradient(160deg,#1d634d,#153a36)}
.action.reverse{border-color:#665232;background:linear-gradient(160deg,#5a472b,#30283a)}
.action.steer{background:linear-gradient(160deg,#294a73,#172b47)}
.action:disabled{opacity:.35;filter:grayscale(.7);cursor:not-allowed}
.footrow{display:flex;justify-content:center;gap:8px;margin-top:10px}
.smallbtn{border:1px solid var(--line);background:var(--panel2);color:var(--text);border-radius:10px;padding:8px 13px;font-weight:700;font-size:12px;cursor:pointer}
.center{display:flex;flex-direction:column;justify-content:center;align-items:center;gap:12px}
.estop{width:min(100%,190px);aspect-ratio:1;border-radius:50%;border:7px solid #762b3a;background:radial-gradient(circle at 40% 28%,#ff6471,#bc1e32 72%);color:white;box-shadow:0 0 0 4px #250f1b,0 10px 32px #0008,inset 0 3px 8px #ffffff55;font-weight:950;font-size:clamp(19px,2.4vw,25px);letter-spacing:.04em;cursor:pointer}
.estop:active{transform:scale(.96)}
.arm{width:100%;max-width:190px;padding:13px 10px;border-radius:12px;border:1px solid #27734f;background:#174b38;color:#b9ffdc;font-weight:800;font-size:14px;cursor:pointer}
.arm.disarmed{border-color:#8b6b2d;background:#47381c;color:#ffe0a0}
.note{text-align:center;color:var(--muted);font-size:11px;line-height:1.4;max-width:190px}
.bottom{display:flex;align-items:center;justify-content:space-between;gap:10px;color:var(--muted);font-size:11px;min-height:22px}
.telemetry{font-variant-numeric:tabular-nums;max-width:65%;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
#rotateHint{display:none}
#event{overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
#modal{position:fixed;inset:0;z-index:10;background:#050914ed;display:flex;align-items:center;justify-content:center;padding:20px}
.login{width:min(100%,360px);background:var(--panel);border:1px solid var(--line);border-radius:18px;padding:24px;box-shadow:0 20px 60px #0009}
.login h2{margin:0 0 8px}.login p{color:var(--muted);font-size:13px;line-height:1.5}
.login input{width:100%;font-size:20px;letter-spacing:.18em;text-align:center;padding:12px;border-radius:10px;border:1px solid var(--line);background:#081222;color:white;margin:10px 0}
.login button{width:100%;padding:12px;border:0;border-radius:10px;background:var(--blue);color:white;font-weight:800;font-size:15px}
#error{color:#ff9aa4;min-height:20px;font-size:12px}
@media (orientation:portrait){#rotateHint{position:fixed;inset:0;z-index:9;display:flex;align-items:center;justify-content:center;text-align:center;padding:30px;background:#07101fee;color:#fff;font-weight:800;font-size:20px}.app{padding:8px;gap:7px}.layout{gap:6px;grid-template-columns:1fr .7fr 1fr}.panel{padding:8px;border-radius:14px}.panelhead{font-size:9px}.bigvalue{font-size:20px}.subvalue{font-size:10px}.action{border-radius:12px}.action small{font-size:9px}.estop{border-width:4px}.arm{font-size:11px;padding:10px 4px}.note{font-size:9px}.smallbtn{padding:7px 6px;font-size:10px}}
@media (max-height:480px){.app{padding-top:5px;padding-bottom:5px;gap:5px}.top{min-height:28px}.panel{padding:8px}.panelhead{margin-bottom:4px}.bigvalue{margin:0 0 3px}.subvalue{margin-bottom:4px}.pad{gap:7px}.footrow{margin-top:5px}.estop{width:min(100%,145px)}.arm{padding:9px}.bottom{min-height:15px}}
/* Compact landscape layout for phones: keep every control inside the visible viewport. */
@media (orientation:landscape) and (max-height:650px){
  .app{height:100%;min-height:0;padding:4px 7px;gap:4px}
  .top{min-height:24px;gap:6px}
  .brand{font-size:clamp(11px,2.1vw,15px)}
  .statusline{gap:4px;font-size:9px}
  .badge{padding:3px 6px}
  .layout{grid-template-columns:minmax(0,1fr) minmax(88px,.62fr) minmax(0,1fr);gap:6px;min-height:0}
  .panel{padding:6px 7px;border-radius:12px}
  .panelhead{font-size:clamp(8px,1.65vh,10px);letter-spacing:.07em;margin:0 0 3px}
  .bigvalue{font-size:clamp(17px,4.6vh,25px);margin:0 0 2px}
  .subvalue{font-size:8px;min-height:10px;margin:0 0 3px}
  .pad{gap:6px}
  .pad.steer{gap:6px}
  .action{font-size:clamp(11px,2vw,15px);gap:3px;border-radius:10px;padding:3px}
  .action .arrow{font-size:clamp(27px,7.5vh,45px)}
  .action small{font-size:clamp(7px,1.35vh,8px);letter-spacing:.03em}
  .footrow{margin-top:4px;gap:5px}
  .smallbtn{padding:5px 6px;font-size:9px;border-radius:8px}
  .center{gap:clamp(3px,1vh,6px)}
  .center .panelhead{margin:0}
  .estop{width:min(100%,18vh,118px);border-width:4px;font-size:clamp(12px,2.3vh,16px);flex:0 0 auto}
  .arm{max-width:none;padding:6px 4px;font-size:clamp(9px,1.7vh,11px);border-radius:9px}
  .note{font-size:clamp(7px,1.35vh,8px);line-height:1.18;max-width:none}
  .bottom{min-height:13px;font-size:8px;gap:6px}
  .telemetry{max-width:58%}
}
@media (orientation:landscape) and (max-height:380px){
  .app{padding:2px 5px;gap:2px}
  .top{min-height:20px}
  .layout{gap:4px;grid-template-columns:minmax(0,1fr) minmax(76px,.55fr) minmax(0,1fr)}
  .panel{padding:4px 5px}
  .estop{width:min(100%,16vh,92px);border-width:3px}
  .arm{padding:4px 3px}
  .note{font-size:7px}
  .bottom{min-height:11px}
}
</style>
</head>
<body>
<div id="modal">
  <form class="login" id="loginForm">
    <h2>Controller access</h2>
    <p>Enter the access PIN printed in the Python server terminal. Use this controller only on a trusted local network.</p>
    <input id="pin" type="password" inputmode="numeric" autocomplete="one-time-code" placeholder="ACCESS PIN" required>
    <div id="error"></div>
    <button type="submit">Connect</button>
  </form>
</div>
<div id="rotateHint">Rotate your device to landscape to use the controls.</div>
<div class="app">
  <header class="top">
    <div class="brand">NTOU <span style="color:#75aef5">ROBOT CONTROL</span></div>
    <div class="statusline">
      <span id="serialBadge" class="badge bad">STM32 OFFLINE</span>
      <span id="armBadge" class="badge warn">DISARMED</span>
    </div>
  </header>
  <main class="layout">
    <section class="panel">
      <div class="panelhead">Drive · left controls</div>
      <div class="bigvalue" id="driveValue">0%</div>
      <div class="subvalue" id="driveLabel">Stopped</div>
      <div class="pad">
        <button class="action forward" id="forwardBtn"><span class="arrow">▲</span><span>FORWARD</span><small>START 20% · STEP 10%</small></button>
        <button class="action reverse" id="reverseBtn"><span class="arrow">▼</span><span>REVERSE</span><small>START 20% · STEP 10%</small></button>
      </div>
      <div class="footrow"><button class="smallbtn" id="driveStopBtn">STOP DRIVE</button></div>
    </section>
    <section class="panel center">
      <div class="panelhead">Safety</div>
      <button class="estop" id="estopBtn">E<br>STOP</button>
      <button class="arm disarmed" id="armBtn">ENABLE CONTROLS</button>
      <div class="note">Emergency stop disables drive and steering. Re-enable explicitly before issuing another command.</div>
    </section>
    <section class="panel">
      <div class="panelhead">Steering · right controls</div>
      <div class="bigvalue" id="steerValue">0°</div>
      <div class="subvalue" id="steerLabel">Centered</div>
      <div class="pad steer">
        <button class="action steer" id="leftBtn"><span class="arrow">◀</span><span>LEFT</span><small>10 · 20 · 30 · 35°</small></button>
        <button class="action steer" id="rightBtn"><span class="arrow">▶</span><span>RIGHT</span><small>10 · 20 · 30 · 35°</small></button>
      </div>
      <div class="footrow"><button class="smallbtn" id="centerBtn">CENTER STEERING</button></div>
    </section>
  </main>
  <footer class="bottom"><span class="telemetry" id="telemetry">Waiting for motor telemetry.</span><span id="event">Waiting for controller connection.</span></footer>
</div>
<script>
const $=id=>document.getElementById(id);
let token=sessionStorage.getItem('ntou_controller_token')||'';
let lastRender='';
let inFlight=false;
async function api(path,body={}){
  const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','Authorization':'Bearer '+token},body:JSON.stringify(body)});
  const d=await r.json().catch(()=>({ok:false,error:'Invalid server response'}));
  if(!r.ok||!d.ok) throw new Error(d.error||('HTTP '+r.status));
  return d;
}
$('loginForm').addEventListener('submit',async e=>{
  e.preventDefault();$('error').textContent='';
  try{const r=await fetch('/api/auth',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({pin:$('pin').value})});const d=await r.json();if(!r.ok||!d.ok)throw new Error(d.error||'Access denied');token=d.token;sessionStorage.setItem('ntou_controller_token',token);$('modal').style.display='none';await refresh();}
  catch(err){$('error').textContent=err.message;}
});
async function action(path,body={}){
  if(inFlight)return;inFlight=true;
  try{const d=await api(path,body);if(d.message)$('event').textContent=d.message;await refresh();}
  catch(err){$('event').textContent='Command error: '+err.message;}
  finally{inFlight=false;}
}
$('forwardBtn').onclick=()=>action('/api/drive',{direction:'forward'});
$('reverseBtn').onclick=()=>action('/api/drive',{direction:'reverse'});
$('driveStopBtn').onclick=()=>action('/api/drive_stop');
$('leftBtn').onclick=()=>action('/api/steer',{direction:'left'});
$('rightBtn').onclick=()=>action('/api/steer',{direction:'right'});
$('centerBtn').onclick=()=>action('/api/center');
$('estopBtn').onclick=()=>action('/api/estop');
$('armBtn').onclick=()=>action('/api/arm');
window.addEventListener('keydown',e=>{
  if($('modal').style.display!=='none')return;
  const k=e.key;
  if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight',' '].includes(k))e.preventDefault();
  if(k==='ArrowUp')$('forwardBtn').click();else if(k==='ArrowDown')$('reverseBtn').click();else if(k==='ArrowLeft')$('leftBtn').click();else if(k==='ArrowRight')$('rightBtn').click();else if(k===' ')$('estopBtn').click();
});
async function refresh(){
  if(!token)return;
  try{
    const r=await fetch('/api/status',{headers:{'Authorization':'Bearer '+token},cache:'no-store'});
    const s=await r.json();if(r.status===401){sessionStorage.removeItem('ntou_controller_token');token='';$('modal').style.display='flex';return;}
    const sig=JSON.stringify(s);
    $('serialBadge').textContent=s.serial_connected?'STM32 CONNECTED':'STM32 OFFLINE';$('serialBadge').className='badge '+(s.serial_connected?'ok':'bad');
    $('armBadge').textContent=s.armed?'ARMED':'DISARMED';$('armBadge').className='badge '+(s.armed?'ok':'warn');
    $('driveValue').textContent=(s.drive_pwm>0?'+':'')+s.drive_pwm+'%';
    $('driveLabel').textContent=s.drive_pwm>0?'Forward':s.drive_pwm<0?'Reverse':'Stopped';
    $('steerValue').textContent=(s.steering_deg>0?'+':'')+s.steering_deg+'°';
    $('steerLabel').textContent=s.steering_deg>0?'Right':s.steering_deg<0?'Left':'Centered';
    const fb=s.feedback||{}; const wheels=fb.wheels||{}; const steering=fb.steering||{};
    const wheelParts=['FL','FR','RR','RL'].map(n=>wheels[n]?(n+' '+String(wheels[n].pwm||'0').replace('%','')+'%'):(n+' --'));
    const steerParts=['SR','SL'].map(n=>steering[n]?(n+' '+(steering[n].pos||'?')+'°'):(n+' --'));
    $('telemetry').textContent='Actual: '+wheelParts.join(' · ')+' | '+steerParts.join(' · ');
    const enabled=s.armed&&s.serial_connected;
    for(const id of ['forwardBtn','reverseBtn','driveStopBtn','leftBtn','rightBtn','centerBtn'])$(id).disabled=!enabled;
    $('armBtn').disabled=!s.serial_connected;
    $('armBtn').textContent=s.armed?'CONTROLS ENABLED':'ENABLE CONTROLS';
    $('armBtn').className='arm '+(s.armed?'':'disarmed');
    if(s.last_event)$('event').textContent=s.last_event;
    if(sig!==lastRender){lastRender=sig;}
  }catch(err){$('serialBadge').textContent='SERVER UNREACHABLE';$('serialBadge').className='badge bad';}
}
async function heartbeat(){
  if(!token)return;
  try{await api('/api/heartbeat');}catch(e){}
}
setInterval(()=>{heartbeat();refresh();},250);
window.addEventListener('pagehide',()=>{
  if(!token)return;
  fetch('/api/estop',{method:'POST',headers:{'Content-Type':'application/json','Authorization':'Bearer '+token},body:JSON.stringify({reason:'Controller page closed'}),keepalive:true}).catch(()=>{});
});
if(token){$('modal').style.display='none';refresh();}
</script>
</body>
</html>'''


class RosWorker:
    def __init__(self, controller: "RobotController") -> None:
        self.controller = controller
        self.running = threading.Event()
        self.running.set()
        rclpy.init()
        self.node = Node("ntou_web_teleop")
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.node.create_subscription(String, "/hardware/serial_feedback", self._fb_cb, 20)
        self.node.create_subscription(Bool, "/hardware/serial_connected", self._conn_cb, 10)
        self.node.create_timer(0.1, self._publish_timer)  # 10 Hz keeps 0.5s watchdog alive
        self.thread = threading.Thread(target=self._spin, name="ros2-spin", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _spin(self) -> None:
        while self.running.is_set() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _conn_cb(self, msg: Bool) -> None:
        self.controller.serial_status(msg.data, "STM32 connected via ROS 2" if msg.data else "STM32 disconnected")

    def _fb_cb(self, msg: String) -> None:
        if not self.controller.serial_connected:
            self.controller.serial_status(True, "STM32 telemetry active via ROS 2")
        self.controller.serial_line(msg.data)

    def publish_cmd(self, drive_pwm: int, steering_deg: int) -> None:
        msg = Twist()
        msg.linear.x = (drive_pwm / float(DRIVE_MAX_PERCENT)) * MAX_LINEAR_X
        msg.angular.z = -(steering_deg / float(MAX_STEER_DEG)) * MAX_ANGULAR_Z
        self.cmd_pub.publish(msg)

    def _publish_timer(self) -> None:
        with self.controller.lock:
            if self.controller.armed and not self.controller.estop:
                self.publish_cmd(self.controller.drive_pwm, self.controller.steering_deg)

    def stop(self) -> None:
        self.publish_cmd(0, 0)
        self.running.clear()
        self.thread.join(timeout=1.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def parse_tokens(line: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for token in line.split():
        if "=" in token:
            key, value = token.split("=", 1)
            values[key] = value
    return values


class RobotController:
    def __init__(self, pin: str) -> None:
        self.pin = pin
        self.lock = threading.RLock()
        self.tokens: set[str] = set()
        self.serial_connected = False
        self.armed = False
        self.estop = True
        self.drive_pwm = 0
        self.steering_deg = 0
        self.last_heartbeat: float | None = None
        self.last_event = "Waiting for STM32 telemetry via ROS 2"
        self.last_command = ""
        self.last_ack = ""
        self.last_rx_time = 0.0
        self.feedback: dict[str, dict[str, Any]] = {"wheels": {}, "steering": {}}
        self.worker = RosWorker(self)
        self.watchdog_running = threading.Event()
        self.watchdog_running.set()
        self.watchdog_thread = threading.Thread(target=self._watchdog, name="web-watchdog", daemon=True)

    def start(self) -> None:
        self.worker.start()
        self.watchdog_thread.start()

    def add_token(self) -> str:
        token = secrets.token_urlsafe(32)
        with self.lock:
            self.tokens.add(token)
        return token

    def valid_token(self, token: str) -> bool:
        with self.lock:
            return any(hmac.compare_digest(token, candidate) for candidate in self.tokens)

    def serial_status(self, connected: bool, message: str) -> None:
        with self.lock:
            changed = self.serial_connected != connected
            self.serial_connected = connected
            if not connected:
                self.armed = False
                self.estop = True
                self.drive_pwm = 0
                self.steering_deg = 0
                self.last_heartbeat = None
            if changed or not connected:
                self.last_event = message
        if changed:
            print(f"[ROS2] {message}", flush=True)

    def serial_line(self, line: str) -> None:
        first = line.split(maxsplit=1)[0] if line.split() else ""
        fields = parse_tokens(line)
        with self.lock:
            self.last_rx_time = time.monotonic()
            if first in WHEELS:
                self.feedback["wheels"][first] = fields
                return
            if first in STEERING:
                self.feedback["steering"][first] = fields
                return
            self.last_event = line
            self.last_ack = line
        if "updated" in line.lower() or "accepted" in line.lower() or "stopped" in line.lower() or "all motors stopped" in line.lower():
            print(f"[STM32 ACK] {line}", flush=True)
        elif "unknown command" in line.lower() or "invalid" in line.lower() or "unavailable" in line.lower():
            print(f"[STM32 ERROR] {line}", flush=True)
        elif line.startswith("RXCMD="):
            print(f"[STM32 DEBUG] {line}", flush=True)

    def set_event(self, message: str) -> None:
        with self.lock:
            self.last_event = message

    def _require_motion_permission(self) -> None:
        if not self.serial_connected:
            raise RuntimeError("STM32 serial connection is offline")
        if not self.armed or self.estop:
            raise RuntimeError("Controls are disarmed. Press ENABLE CONTROLS first.")

    def arm(self) -> str:
        with self.lock:
            if not self.serial_connected:
                raise RuntimeError("STM32 serial connection is offline")
            if self.armed and not self.estop:
                self.last_heartbeat = time.monotonic()
                return "Controls are already enabled"
            self.drive_pwm = 0
            self.steering_deg = 0
            self.worker.publish_cmd(0, 0)
            self.estop = False
            self.armed = True
            self.last_heartbeat = time.monotonic()
            self.last_event = "Controls enabled; motors remain stopped until a direction button is pressed"
            return self.last_event

    def emergency_stop(self, reason: str = "Emergency stop pressed") -> str:
        with self.lock:
            can_send = self.serial_connected
            self.drive_pwm = 0
            self.steering_deg = 0
            self.armed = False
            self.estop = True
            self.last_heartbeat = None
            self.last_event = reason + ("; zero cmd_vel sent" if can_send else "; hardware offline")
            if can_send:
                self.worker.publish_cmd(0, 0)
            return self.last_event

    def drive_step(self, direction: str) -> str:
        with self.lock:
            self._require_motion_permission()
            current = self.drive_pwm
            if direction == "forward":
                if current < 0:
                    new = 0
                    message = "Direction change: stopped. Tap FORWARD again to start forward at 20%."
                else:
                    new = DRIVE_START_PERCENT if current == 0 else min(current + DRIVE_STEP_PERCENT, DRIVE_MAX_PERCENT)
                    message = f"Forward drive set to {new}%"
            elif direction == "reverse":
                if current > 0:
                    new = 0
                    message = "Direction change: stopped. Tap REVERSE again to start reverse at 20%."
                else:
                    new = -DRIVE_START_PERCENT if current == 0 else max(current - DRIVE_STEP_PERCENT, -DRIVE_MAX_PERCENT)
                    message = f"Reverse drive set to {abs(new)}%"
            else:
                raise RuntimeError("Unknown drive direction")
            self.drive_pwm = new
            self.worker.publish_cmd(self.drive_pwm, self.steering_deg)
            self.last_event = message
            return message

    def drive_stop(self) -> str:
        with self.lock:
            self._require_motion_permission()
            self.drive_pwm = 0
            self.worker.publish_cmd(0, self.steering_deg)
            self.last_event = "Drive motors stopped; steering control remains enabled"
            return self.last_event

    def steer_step(self, direction: str) -> str:
        with self.lock:
            self._require_motion_permission()
            current = self.steering_deg
            if direction == "right":
                if current < 0:
                    new = 10
                elif current == 0:
                    new = 10
                else:
                    new = next((step for step in STEERING_STEPS if step > current), current)
            elif direction == "left":
                if current > 0:
                    new = -10
                elif current == 0:
                    new = -10
                else:
                    magnitude = abs(current)
                    next_step = next((step for step in STEERING_STEPS if step > magnitude), magnitude)
                    new = -next_step
            else:
                raise RuntimeError("Unknown steering direction")
            self.steering_deg = new
            self.worker.publish_cmd(self.drive_pwm, self.steering_deg)
            self.last_event = f"Steering target set to {new:+d} degrees"
            return self.last_event

    def center_steering(self) -> str:
        with self.lock:
            self._require_motion_permission()
            self.steering_deg = 0
            self.worker.publish_cmd(self.drive_pwm, 0)
            self.last_event = "Steering target set to centre (0 degrees)"
            return self.last_event

    def heartbeat(self) -> None:
        with self.lock:
            if self.armed and not self.estop:
                self.last_heartbeat = time.monotonic()

    def _watchdog(self) -> None:
        while self.watchdog_running.is_set():
            time.sleep(0.1)
            now = time.monotonic()
            with self.lock:
                expired = (self.armed and self.last_heartbeat is not None and
                           now - self.last_heartbeat > HEARTBEAT_TIMEOUT_S)
                rx_stale = (self.serial_connected and self.last_rx_time > 0.0 and
                            now - self.last_rx_time > 1.5)
            if rx_stale:
                self.serial_status(False, "STM32 telemetry timeout (>1.5s)")
            if expired:
                self.emergency_stop("Browser heartbeat lost; safety stop triggered")
                print("[SAFETY] Browser heartbeat lost; zero cmd_vel sent and controller disarmed.", flush=True)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "serial_connected": self.serial_connected,
                "armed": self.armed,
                "estop": self.estop,
                "drive_pwm": self.drive_pwm,
                "steering_deg": self.steering_deg,
                "last_event": self.last_event,
                "last_command": self.last_command,
                "last_ack": self.last_ack,
                "feedback": self.feedback,
            }

    def shutdown(self) -> None:
        self.emergency_stop("Server shutting down")
        time.sleep(0.1)
        self.watchdog_running.clear()
        self.watchdog_thread.join(timeout=0.5)
        self.worker.stop()


class ControllerHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class RequestHandler(BaseHTTPRequestHandler):
    server_version = "NTOUController/1.0"

    @property
    def controller(self) -> RobotController:
        return self.server.controller  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        # Keep routine browser polling out of the terminal log.
        if self.path.startswith("/api/"):
            return
        super().log_message(fmt, *args)

    def _json(self, status: int, payload: dict[str, Any]) -> None:
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length > 4096:
            raise ValueError("Request body too large")
        raw = self.rfile.read(length) if length else b"{}"
        value = json.loads(raw.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("JSON body must be an object")
        return value

    def _authorized(self) -> bool:
        header = self.headers.get("Authorization", "")
        token = header[7:] if header.startswith("Bearer ") else ""
        if not self.controller.valid_token(token):
            self._json(401, {"ok": False, "error": "Not authenticated"})
            return False
        return True

    def do_GET(self) -> None:
        if self.path == "/" or self.path.startswith("/?"):
            data = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(data)
            return
        if self.path == "/api/status":
            if not self._authorized():
                return
            self._json(200, {"ok": True, **self.controller.snapshot()})
            return
        self._json(404, {"ok": False, "error": "Not found"})

    def do_POST(self) -> None:
        try:
            body = self._body()
        except (ValueError, json.JSONDecodeError, UnicodeDecodeError) as exc:
            self._json(400, {"ok": False, "error": str(exc)})
            return

        if self.path == "/api/auth":
            supplied = str(body.get("pin", ""))
            if not hmac.compare_digest(supplied, self.controller.pin):
                self._json(403, {"ok": False, "error": "Incorrect PIN"})
                return
            token = self.controller.add_token()
            self._json(200, {"ok": True, "token": token})
            return

        if not self._authorized():
            return
        try:
            if self.path == "/api/heartbeat":
                self.controller.heartbeat()
                result = "Heartbeat received"
            elif self.path == "/api/arm":
                result = self.controller.arm()
            elif self.path == "/api/estop":
                reason = str(body.get("reason", "Emergency stop pressed"))[:100]
                result = self.controller.emergency_stop(reason)
            elif self.path == "/api/drive":
                result = self.controller.drive_step(str(body.get("direction", "")))
            elif self.path == "/api/drive_stop":
                result = self.controller.drive_stop()
            elif self.path == "/api/steer":
                result = self.controller.steer_step(str(body.get("direction", "")))
            elif self.path == "/api/center":
                result = self.controller.center_steering()
            else:
                self._json(404, {"ok": False, "error": "Not found"})
                return
            self._json(200, {"ok": True, "message": result, **self.controller.snapshot()})
        except RuntimeError as exc:
            self._json(409, {"ok": False, "error": str(exc)})


def get_lan_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
        
def main() -> int:
    parser = argparse.ArgumentParser(description="NTOU ROS 2 browser-based drive and steering controller")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind address (default: 0.0.0.0 for LAN access)")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP port (default: 8080)")
    parser.add_argument("--pin", default="", help="Access PIN; generated automatically if omitted")
    args, _ = parser.parse_known_args()

    pin = args.pin.strip() or f"{secrets.randbelow(1_000_000):06d}"
    controller = RobotController(pin)
    server = ControllerHTTPServer((args.host, args.http_port), RequestHandler)
    server.controller = controller  # type: ignore[attr-defined]
    controller.start()

    lan_ip = get_lan_ip()
    print("\nNTOU ROBOT WEB CONTROLLER (ROS 2)", flush=True)
    print(f"Web UI: http://127.0.0.1:{args.http_port}", flush=True)
    print(f"LAN access: http://{lan_ip}:{args.http_port}", flush=True)
    print(f"Access PIN: {pin}", flush=True)
    print("Controller starts DISARMED. Browser heartbeat timeout: 1.25 s.\n", flush=True)

    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\nShutting down controller...", flush=True)
    finally:
        server.server_close()
        controller.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

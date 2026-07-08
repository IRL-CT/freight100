#!/usr/bin/env python3
"""
Web teleop controller for the Freight100 (Robust.AI overlay).

Serves a self-contained joystick web page and republishes browser commands to
/robust/nav_cmd_vel (the movement-mux input) at 10 Hz.

Safety model:
  - The browser must STREAM commands (~10 Hz keepalive) while a control is held.
    If commands stop arriving for WATCHDOG_S (tab closed, Wi-Fi drop, control
    released), the robot is commanded to zero immediately.
  - Speeds are clamped server-side to MAX_LIN / MAX_ANG regardless of client.
  - The hardware runstop always overrides everything in hardware.

Run on the robot:
    source /opt/ros/noetic/setup.bash
    python3 web_teleop.py
Then open  http://<robot-ip>:8090  from a device on the same network.
"""
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from fetch_driver_msgs.msg import RobotState
from power_msgs.msg import BatteryState

PORT = 8090
MAX_LIN = 0.30        # m/s   server-side hard cap
MAX_ANG = 0.60        # rad/s server-side hard cap
WATCHDOG_S = 0.4      # zero the robot if no command within this window
PUB_HZ = 10

state = {
    "vx": 0.0, "wz": 0.0, "t_cmd": 0.0,      # latest browser command
    "runstopped": None, "ready": None, "faulted": None,
    "charge": None, "charging": None,
    "ox": 0.0, "oy": 0.0,
    "active": False,                           # watchdog verdict, for UI
}
lock = threading.Lock()


def clamp(v, lim):
    return max(-lim, min(lim, v))


def robot_state_cb(msg):
    with lock:
        state["runstopped"] = msg.runstopped
        state["ready"] = msg.ready
        state["faulted"] = msg.faulted


def battery_cb(msg):
    with lock:
        state["charge"] = msg.charge_level
        state["charging"] = msg.is_charging


def odom_cb(msg):
    p = msg.pose.pose.position
    with lock:
        state["ox"], state["oy"] = p.x, p.y


def publisher_loop(pub):
    """10 Hz: republish latest command while fresh; publish zero when stale."""
    rate = rospy.Rate(PUB_HZ)
    was_active = False
    while not rospy.is_shutdown():
        now = time.monotonic()
        with lock:
            fresh = (now - state["t_cmd"]) < WATCHDOG_S
            vx = clamp(state["vx"], MAX_LIN) if fresh else 0.0
            wz = clamp(state["wz"], MAX_ANG) if fresh else 0.0
            state["active"] = fresh and (abs(vx) > 1e-4 or abs(wz) > 1e-4)
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        if fresh and (abs(vx) > 1e-4 or abs(wz) > 1e-4):
            pub.publish(cmd)
            was_active = True
        elif was_active:
            pub.publish(Twist())   # single zero burst on release/timeout
            was_active = False
        rate.sleep()


PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Freight100 Teleop</title>
<style>
  :root { --bg:#12151a; --panel:#1c2129; --fg:#e6e9ee; --dim:#8a93a3;
          --ok:#38b26c; --warn:#e0a534; --bad:#e05252; --acc:#3d8bfd; }
  * { box-sizing:border-box; margin:0; -webkit-user-select:none; user-select:none;
      -webkit-touch-callout:none; touch-action:manipulation; }
  body { background:var(--bg); color:var(--fg);
         font:15px/1.45 system-ui,-apple-system,sans-serif;
         display:flex; flex-direction:column; align-items:center;
         min-height:100vh; padding:14px; gap:12px; }
  h1 { font-size:17px; font-weight:600; letter-spacing:.02em; }
  h1 small { color:var(--dim); font-weight:400; }
  #status { display:flex; gap:8px; flex-wrap:wrap; justify-content:center; }
  .chip { background:var(--panel); border-radius:999px; padding:4px 12px;
          font-size:13px; color:var(--dim); }
  .chip b { color:var(--fg); }
  .chip.ok b { color:var(--ok); } .chip.warn b { color:var(--warn); }
  .chip.bad b { color:var(--bad); }
  #pad { width:min(78vw,300px); height:min(78vw,300px); background:var(--panel);
         border-radius:50%; position:relative; border:2px solid #2a3140;
         touch-action:none; }
  #knob { width:34%; height:34%; background:var(--acc); border-radius:50%;
          position:absolute; left:33%; top:33%; box-shadow:0 3px 14px #0007;
          transition:background .15s; }
  #pad.live #knob { background:var(--ok); }
  #vel { font-size:13px; color:var(--dim); font-variant-numeric:tabular-nums; }
  #stop { width:min(78vw,300px); padding:16px; font-size:19px; font-weight:700;
          color:#fff; background:var(--bad); border:0; border-radius:12px;
          letter-spacing:.06em; cursor:pointer; }
  #stop:active { filter:brightness(.8); }
  .row { display:flex; gap:10px; align-items:center; width:min(78vw,300px);
         font-size:13px; color:var(--dim); }
  .row input { flex:1; accent-color:var(--acc); }
  .keys { color:var(--dim); font-size:12.5px; text-align:center; }
  kbd { background:#2a3140; border-radius:4px; padding:1px 6px; font-size:12px; }
</style></head><body>

<h1>Freight100 Teleop <small>· freight1794</small></h1>
<div id="status">
  <span class="chip" id="c_run">runstop <b>—</b></span>
  <span class="chip" id="c_ready">ready <b>—</b></span>
  <span class="chip" id="c_batt">battery <b>—</b></span>
  <span class="chip" id="c_odom">odom <b>—</b></span>
</div>

<div id="pad"><div id="knob"></div></div>
<div id="vel">vx 0.00 m/s · wz 0.00 rad/s</div>

<div class="row">speed
  <input type="range" id="scale" min="10" max="100" value="50">
  <span id="scalev">50%</span>
</div>

<button id="stop">■ STOP</button>
<div class="keys">Keys: <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> or arrows — hold to drive,
release to stop. <kbd>Space</kbd> = STOP.</div>

<script>
const MAX_LIN = %MAX_LIN%, MAX_ANG = %MAX_ANG%;
const pad = document.getElementById('pad'), knob = document.getElementById('knob');
const scaleEl = document.getElementById('scale');
let vec = {x:0, y:0};        // -1..1 (y fwd+, x right+)
let dragging = false, keys = {};

function scale() { return scaleEl.value / 100; }
document.getElementById('scale').oninput = () =>
  document.getElementById('scalev').textContent = scaleEl.value + '%';

/* --- joystick pad --- */
function setKnob(nx, ny) {
  knob.style.left = (33 + nx*33) + '%';
  knob.style.top  = (33 - ny*33) + '%';
}
function padEvent(e) {
  const r = pad.getBoundingClientRect();
  const t = e.touches ? e.touches[0] : e;
  let nx = ((t.clientX - r.left) / r.width - 0.5) * 2;
  let ny = -((t.clientY - r.top) / r.height - 0.5) * 2;
  const m = Math.hypot(nx, ny);
  if (m > 1) { nx /= m; ny /= m; }
  vec = {x:nx, y:ny};
  setKnob(nx, ny);
}
function padEnd() { dragging = false; vec = {x:0, y:0}; setKnob(0, 0); }
pad.addEventListener('pointerdown', e => { dragging = true; pad.setPointerCapture(e.pointerId); padEvent(e); });
pad.addEventListener('pointermove', e => { if (dragging) padEvent(e); });
pad.addEventListener('pointerup', padEnd);
pad.addEventListener('pointercancel', padEnd);

/* --- keyboard --- */
addEventListener('keydown', e => {
  if (e.code === 'Space') { stopAll(); e.preventDefault(); return; }
  keys[e.code] = true;
});
addEventListener('keyup', e => { keys[e.code] = false; });
function keyVec() {
  let y = (keys.KeyW || keys.ArrowUp ? 1 : 0) - (keys.KeyS || keys.ArrowDown ? 1 : 0);
  let x = (keys.KeyD || keys.ArrowRight ? 1 : 0) - (keys.KeyA || keys.ArrowLeft ? 1 : 0);
  return {x, y};
}

function stopAll() { padEnd(); keys = {}; fetch('/cmd', {method:'POST', body:'{"vx":0,"wz":0}'}); }
document.getElementById('stop').onclick = stopAll;

/* --- 10 Hz command stream (keepalive = deadman) --- */
setInterval(() => {
  const k = keyVec();
  const ax = dragging ? vec.x : k.x, ay = dragging ? vec.y : k.y;
  const vx = ay * MAX_LIN * scale();
  const wz = -ax * MAX_ANG * scale();
  const live = Math.abs(vx) > 1e-3 || Math.abs(wz) > 1e-3;
  pad.classList.toggle('live', live);
  document.getElementById('vel').textContent =
    'vx ' + vx.toFixed(2) + ' m/s · wz ' + wz.toFixed(2) + ' rad/s';
  if (live) fetch('/cmd', {method:'POST', body:JSON.stringify({vx, wz})}).catch(()=>{});
}, 100);

/* --- status poll --- */
function chip(id, label, val, cls) {
  const el = document.getElementById(id);
  el.innerHTML = label + ' <b>' + val + '</b>';
  el.className = 'chip ' + (cls || '');
}
setInterval(async () => {
  try {
    const s = await (await fetch('/status')).json();
    chip('c_run', 'runstop', s.runstopped ? 'ENGAGED' : 'released',
         s.runstopped ? 'bad' : 'ok');
    chip('c_ready', 'ready', s.ready ? 'yes' : 'no', s.ready ? 'ok' : 'bad');
    chip('c_batt', 'battery',
         s.charge == null ? '—' : Math.round(s.charge*100) + '%' + (s.charging ? ' ⚡' : ''),
         s.charge != null && s.charge < 0.15 ? 'warn' : '');
    chip('c_odom', 'odom', s.ox.toFixed(2) + ', ' + s.oy.toFixed(2));
  } catch(e) {}
}, 1000);
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):   # quiet
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            page = PAGE.replace("%MAX_LIN%", str(MAX_LIN)).replace("%MAX_ANG%", str(MAX_ANG))
            self._send(200, page, "text/html; charset=utf-8")
        elif self.path == "/status":
            with lock:
                self._send(200, json.dumps({k: state[k] for k in
                    ("runstopped", "ready", "faulted", "charge", "charging",
                     "ox", "oy", "active")}))
        else:
            self._send(404, '{"err":"not found"}')

    def do_POST(self):
        if self.path != "/cmd":
            self._send(404, '{"err":"not found"}')
            return
        try:
            n = int(self.headers.get("Content-Length", 0))
            d = json.loads(self.rfile.read(n) or b"{}")
            vx = clamp(float(d.get("vx", 0.0)), MAX_LIN)
            wz = clamp(float(d.get("wz", 0.0)), MAX_ANG)
            if not (math.isfinite(vx) and math.isfinite(wz)):
                raise ValueError("non-finite")
        except Exception:
            self._send(400, '{"err":"bad command"}')
            return
        with lock:
            state["vx"], state["wz"] = vx, wz
            state["t_cmd"] = time.monotonic()
        self._send(200, '{"ok":true}')


def main():
    rospy.init_node("web_teleop", anonymous=True)
    pub = rospy.Publisher("/robust/nav_cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/robot_state", RobotState, robot_state_cb)
    rospy.Subscriber("/battery_state", BatteryState, battery_cb)
    rospy.Subscriber("/odom", Odometry, odom_cb)

    threading.Thread(target=publisher_loop, args=(pub,), daemon=True).start()

    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"Freight100 web teleop on http://0.0.0.0:{PORT}")
    print(f"caps: {MAX_LIN} m/s lin, {MAX_ANG} rad/s ang; watchdog {WATCHDOG_S*1000:.0f} ms")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        pub.publish(Twist())   # parting zero


if __name__ == "__main__":
    main()

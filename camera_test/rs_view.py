#!/usr/bin/env python3
"""
Live viewer for the Intel RealSense D435i on this robot — shows what the
camera sees (color + colorized depth side by side).

The camera is on a USB 2.0 link (enumerates as "2.1", 480 Mbps), so streams
are fixed at the USB2-safe profile 640x480 @ 30 fps. Frames are read directly
from the device with pyrealsense2 — the robot's multicam ROS topics
(/left_cam/..., /right_cam/...) are stale registrations and publish nothing,
so the device is free to open.

Modes (auto-picked, or forced with a flag):
    window    OpenCV window on the robot's monitor (needs DISPLAY; q/Esc quits)
    web       MJPEG stream in a browser, same idea as ../web_teleop.py
    snapshot  save one color/depth image pair and exit

Run on the robot:
    python3 rs_view.py                 # window if DISPLAY is set, else web
    python3 rs_view.py --web           # then open http://<robot-ip>:8091
    python3 rs_view.py --snapshot      # writes rs_color.png / rs_depth.png
    python3 rs_view.py --duration 10   # auto-exit after N seconds (testing)

Overlay: FPS, center-pixel distance (crosshair; "--" when the surface is
closer than the ~0.3 m minimum range or gives no return), serial and USB mode.
"""
import argparse
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import numpy as np
import pyrealsense2 as rs
if hasattr(rs, "pyrealsense2"):        # newer pip wheels nest the module
    rs = rs.pyrealsense2
import cv2

W, H, FPS = 640, 480, 30               # USB2-safe profile (verified on this unit)
PORT = 8091                            # web_teleop.py owns 8090
JPEG_QUALITY = 80


class Camera:
    """Owns the RealSense pipeline; a background thread keeps the latest
    rendered frame available to any number of consumers."""

    def __init__(self):
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
        cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
        profile = self.pipe.start(cfg)
        dev = profile.get_device()
        self.serial = dev.get_info(rs.camera_info.serial_number)
        self.usb = dev.get_info(rs.camera_info.usb_type_descriptor)
        self.colorizer = rs.colorizer()
        self.lock = threading.Lock()
        self.frame_ready = threading.Condition(self.lock)
        self.frame = None              # latest color|depth composite (BGR)
        self.center_m = 0.0
        self.fps = 0.0
        self.running = True
        self.error = None
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        t_last, n = time.time(), 0
        droughts = 0
        try:
            while self.running:
                try:
                    fs = self.pipe.wait_for_frames(timeout_ms=5000)
                    droughts = 0
                except RuntimeError:
                    # right after a reopen the USB2 link can stall for several
                    # seconds before frames flow — tolerate a short drought
                    droughts += 1
                    if droughts >= 3 or not self.running:
                        raise
                    continue
                depth, color = fs.get_depth_frame(), fs.get_color_frame()
                if not depth or not color:
                    continue
                center_m = depth.get_distance(W // 2, H // 2)
                color_img = np.asanyarray(color.get_data())
                depth_img = np.asanyarray(
                    self.colorizer.colorize(depth).get_data())[:, :, ::-1]
                composite = np.hstack((color_img, np.ascontiguousarray(depth_img)))
                n += 1
                now = time.time()
                if now - t_last >= 1.0:
                    self.fps, n, t_last = n / (now - t_last), 0, now
                self._annotate(composite, center_m)
                with self.frame_ready:
                    self.frame = composite
                    self.center_m = center_m
                    self.frame_ready.notify_all()
        except Exception as e:
            self.error = e
            with self.frame_ready:
                self.running = False
                self.frame_ready.notify_all()

    def _annotate(self, img, center_m):
        dist = f"{center_m:.2f} m" if center_m > 0 else "--"
        for x0, label in ((0, "color"), (W, "depth")):
            cv2.putText(img, label, (x0 + 8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cx, cy = x0 + W // 2, H // 2
            cv2.drawMarker(img, (cx, cy), (255, 255, 255),
                           cv2.MARKER_CROSS, 16, 1, cv2.LINE_AA)
            cv2.putText(img, dist, (cx + 12, cy - 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)
        footer = f"D435i {self.serial}  USB {self.usb}  {W}x{H}@{FPS}  {self.fps:.0f} fps"
        cv2.putText(img, footer, (8, H - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)

    def wait_frame(self, timeout=5.0):
        """Latest cached frame, blocking only until the first one arrives;
        None on stop/timeout. Can lag a stalled camera by up to ~5 s — use
        next_frame() when freshness matters."""
        with self.frame_ready:
            if not self.frame_ready.wait_for(
                    lambda: self.frame is not None or not self.running, timeout):
                return None
            return None if not self.running else self.frame.copy()

    def latest(self, timeout=5.0):
        """(frame, center_m) captured under one lock acquisition, so the
        distance always matches the returned image; (None, 0.0) on stop/timeout."""
        with self.frame_ready:
            if not self.frame_ready.wait_for(
                    lambda: self.frame is not None or not self.running, timeout) \
                    or not self.running:
                return None, 0.0
            return self.frame.copy(), self.center_m

    def next_frame(self, timeout=5.0):
        """Like wait_frame but always waits for a fresh frame (for streaming)."""
        with self.frame_ready:
            prev = self.frame
            if not self.frame_ready.wait_for(
                    lambda: (self.frame is not None and self.frame is not prev)
                    or not self.running, timeout):
                return None
            return None if not self.running else self.frame.copy()

    def stop(self):
        self.running = False
        with self.frame_ready:
            self.frame_ready.notify_all()
        self.thread.join(timeout=7)
        try:
            self.pipe.stop()
        except RuntimeError:
            pass


def page_html(token):
    q = f"?token={token}" if token else ""
    return f"""<!doctype html><html><head><meta charset="utf-8">
<title>D435i live — freight1794</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{{margin:0;background:#111;color:#ddd;font:14px sans-serif;text-align:center}}
img{{max-width:100%;height:auto}}</style></head>
<body><p>RealSense D435i — color | depth &nbsp; (<a style="color:#8cf"
href="/shot.jpg{q}">single frame</a>)</p><img src="/stream.mjpg{q}"></body></html>
"""


def make_handler(cam, token=None, stream_fps=0):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):          # keep the terminal quiet
            pass

        def _send_jpeg(self, frame):
            ok, buf = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok:
                self.send_error(500, "encode failed")
                return None
            return buf.tobytes()

        def do_GET(self):
            try:
                self._get()
            except OSError:
                pass                     # client went away mid-response

        def _get(self):
            url = urlsplit(self.path)
            path = url.path
            if token:
                given = parse_qs(url.query).get("token", [""])[0]
                if given != token:
                    self.send_error(403, "missing or wrong token "
                                         "(append ?token=... to the URL)")
                    return
            if path in ("/", "/index.html"):
                body = page_html(token).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/shot.jpg":
                frame = cam.next_frame()   # fresh frame, not the cached one
                if frame is None:
                    self.send_error(503, "camera not streaming")
                    return
                data = self._send_jpeg(frame)
                if data is None:
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            elif path == "/stream.mjpg":
                frame = cam.next_frame()   # prefetch so a dead camera 503s
                if frame is None:
                    self.send_error(503, "camera not streaming")
                    return
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                interval = 1.0 / stream_fps if stream_fps else 0
                t_sent = 0.0
                while frame is not None:
                    if interval and time.time() - t_sent < interval:
                        frame = cam.next_frame()   # drop frame, keep pace
                        continue
                    t_sent = time.time()
                    data = self._send_jpeg(frame)
                    if data is None:
                        break
                    self.wfile.write(b"--frame\r\n"
                                     b"Content-Type: image/jpeg\r\n"
                                     b"Content-Length: "
                                     + str(len(data)).encode() + b"\r\n\r\n")
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                    frame = cam.next_frame()
            else:
                self.send_error(404)
    return Handler


def run_window(cam, duration):
    print("Viewer window open on the robot's display — q or Esc to quit.")
    cv2.namedWindow("D435i", cv2.WINDOW_AUTOSIZE)
    t0 = time.time()
    while cam.running:
        frame = cam.wait_frame(timeout=1.0)
        if frame is not None:
            cv2.imshow("D435i", frame)
        if cv2.waitKey(30) & 0xFF in (ord("q"), 27):
            break
        if duration and time.time() - t0 >= duration:
            break
        if cv2.getWindowProperty("D435i", cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyAllWindows()


def run_web(cam, duration, token=None, stream_fps=0):
    server = ThreadingHTTPServer(("0.0.0.0", PORT),
                                 make_handler(cam, token, stream_fps))
    q = f"?token={token}" if token else ""
    print(f"Serving on http://localhost:{PORT}/{q}  "
          f"(from Wi-Fi: http://<robot-ip>:{PORT}/{q}, see `ip -4 addr show wlan0`)")
    if token:
        print("Access token required — URLs without ?token=... get 403.")
    print("Ctrl+C to stop.")
    if duration:
        timer = threading.Timer(duration, server.shutdown)
        timer.daemon = True            # never block interpreter exit on Ctrl+C
        timer.start()
    # if the camera dies, take the server down with it so main reports the error
    threading.Thread(target=lambda: (cam.thread.join(), server.shutdown()),
                     daemon=True).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def run_snapshot(cam):
    frame, center_m = cam.latest()
    if frame is None:
        return False
    out = os.path.dirname(os.path.abspath(__file__))
    for name, img in (("rs_color.png", frame[:, :W]),
                      ("rs_depth.png", frame[:, W:])):
        path = os.path.join(out, name)
        cv2.imwrite(path, img)
        print("wrote", path)
    print(f"center distance: {center_m:.3f} m"
          + ("  (0.0 = closer than min range / no return)"
             if center_m == 0 else ""))
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--window", action="store_true", help="force OpenCV window")
    mode.add_argument("--web", action="store_true", help="force MJPEG web server")
    mode.add_argument("--snapshot", action="store_true",
                      help="save one color/depth pair and exit")
    ap.add_argument("--duration", type=float, default=0,
                    help="auto-exit after N seconds (0 = run until quit)")
    ap.add_argument("--token", default=None,
                    help="web mode: require ?token=... on every request "
                         "(use when exposing beyond the LAN)")
    ap.add_argument("--stream-fps", type=float, default=0,
                    help="web mode: cap MJPEG frame rate (0 = camera rate; "
                         "10-12 recommended over the internet)")
    args = ap.parse_args()

    try:
        cam = Camera()
    except RuntimeError as e:
        sys.exit(f"Could not open the RealSense: {e}\n"
                 "Is it plugged in (lsusb | grep 8086:0b3a) and not in use "
                 "by another process?")
    print(f"D435i {cam.serial}, firmware OK, USB {cam.usb} "
          f"({'USB2 mode - reduced profiles' if cam.usb.startswith('2') else 'USB3'})")

    try:
        if args.snapshot:
            ok = run_snapshot(cam)
        elif args.web or (not args.window and not os.environ.get("DISPLAY")):
            if not args.web:
                print("No DISPLAY — falling back to web mode.")
            run_web(cam, args.duration, args.token, args.stream_fps)
            ok = True
        else:
            run_window(cam, args.duration)
            ok = True
    except KeyboardInterrupt:
        ok = True
    finally:
        cam.stop()
    if cam.error is not None:
        sys.exit(f"Camera stream died: {cam.error}")
    if not ok:
        sys.exit("No frames received from the camera.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:      # Ctrl+C during teardown — exit quietly
        sys.exit(130)

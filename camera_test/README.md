# Camera Test — Intel RealSense D435i

> Companion to the [main system overview](../README.md). Records the USB check done on
> 2026-08-21 and provides [`rs_view.py`](rs_view.py), a standalone viewer that shows what
> the camera sees (color + colorized depth).

---

## 1. USB Connection Check (2026-08-21)

| Property | Value |
|---|---|
| Device | Intel(R) RealSense(TM) Depth Camera **D435i** (`8086:0b3a`) |
| Serial | `112322071745` |
| Firmware | `5.13.0.50` |
| Kernel nodes | `/dev/video0`–`/dev/video5` (world-writable, plugdev ACL — no perms issue) |
| **Negotiated link** | **480 Mbps = USB 2.1 mode** (`/sys/bus/usb/devices/1-7/speed`) |

⚠️ **The camera is on the USB 2.0 root hub** (Bus 001; the USB 3.0 hub Bus 002 has nothing
on it). It works, but in USB2 fallback mode: max ~640x480 @ 30 fps for depth and color, and
some profiles are unavailable. If full USB3 rates are ever needed (848x480/1280x720, higher
fps), move it to a USB 3 port with a USB 3 cable — after replugging, `lsusb -t` should show
it under the `xhci` 5000M bus and `rs_view.py` will print `USB 3.x` at startup.

Note the [main README §3](../README.md) describes a multi-camera rig (`left_cam`,
`right_cam`, `top_mounted_camera_d435`); as of this check **only this one D435i is on USB**,
and the `/left_cam/*` / `/right_cam/*` ROS topics are stale registrations publishing
nothing. `rs_view.py` therefore opens the device directly — no conflict with the running
Robust.AI stack.

## 2. Dependencies (installed 2026-08-21)

- `pyrealsense2` — installed with `pip3 install --user pyrealsense2` (imports nested as
  `pyrealsense2.pyrealsense2`; the script handles this).
- `opencv-python` 4.5.5 and `numpy` — were already present system-wide.

## 3. Show What the Camera Sees

```bash
cd ~/Desktop/freight100/camera_test
python3 rs_view.py                 # window on the robot's monitor (q or Esc quits)
python3 rs_view.py --web           # browser view: http://<robot-ip>:8091
python3 rs_view.py --snapshot      # save rs_color.png / rs_depth.png and exit
python3 rs_view.py --duration 10   # any mode, auto-exit after 10 s
```

With no flags it opens an OpenCV window if `DISPLAY` is set, otherwise it falls back to the
web server. The view is color | depth side by side with a center-distance crosshair
("`--`" = surface closer than the ~0.3 m minimum range or no return) and an FPS/serial/USB
footer. Web mode serves `/` (viewer page), `/stream.mjpg` (MJPEG), `/shot.jpg` (one frame);
port 8091 (teleop owns 8090).

Verified on this unit 2026-08-21: all three modes, sustained **30 fps** end-to-end over the
MJPEG stream, clean shutdown. Only one process can hold the camera at a time — a second
instance fails at startup with "Could not open the RealSense".

---

*See also [`../OPERATING_TESTS.md`](../OPERATING_TESTS.md) for driving the base.*

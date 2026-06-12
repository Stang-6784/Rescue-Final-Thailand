# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Teleoperation control software for a search-and-rescue robot. The robot's low-level hardware (Teensy MCU, AK45 BLDC drive motors, 8 arm/flipper servos) lives on a **Raspberry Pi**; this repo is the **Windows operator-side** stack plus a browser UI. Code comments and logs are in Thai.

There is no build system, no test suite, and no `requirements.txt`. Everything is run directly with `python`.

## Running the components

These are independent processes; a full session runs them in separate terminals.

```bash
# Operator bridge: HTTP(8766) + WebSocket(8765) + TCP client to the Pi(9000).
# Auto-opens the browser UI at http://localhost:8766/ui/control.html
python rescue.py

# Vision server: Flask on :5000 — RTSP camera, motion detection, QR scan, YOLO11 AI
python app_yolo11.py

# Standalone AK45 motor test client (host defaults to 192.168.1.100)
python rescue_v2.py <PI_IP>
```

Runtime dependencies (install ad hoc): `websockets`, `flask`, `flask_cors`, `opencv-python` (`cv2`), `numpy`, `ultralytics` (YOLO; optional — `app_yolo11.py` degrades gracefully if missing).

## Architecture & data flow

```
Browser (ui/control.html + script.js)
   │  ├── WebSocket :8765  ──►  rescue.py  ──TCP :9000──►  Raspberry Pi  ──►  Teensy / AK45 / servos
   │  └── HTTP/REST :5000  ──►  app_yolo11.py  (video_feed, /status, /scan_qr, /ai/*)
   │
rescue.py also polls app_yolo11.py :5000/status for QR results (QRWorker)
```

- **`rescue.py`** — the central operator bridge. Single process running three servers via threads + asyncio:
  - HTTP server (`_http_server`, port 8766) serves the static `ui/` folder with permissive CORS/iframe headers.
  - WebSocket server (port 8765) is the browser↔robot command channel. `RobotController.handle_ws` is the giant dispatch switch for every command type (`move_start`, `servo_set`, `posture`, `motor_*`, etc.).
  - TCP client to the Pi (port 9000): `_send_pi` writes newline-delimited JSON; `_pi_recv_thread` reads it back. Connection is auto-reconnecting (`_pi_connect_thread`).
  - State is pushed to all browsers at `STATE_PUSH_HZ` (20Hz) plus on every event; driving keys are re-sent to the Pi at 50Hz from `_send_loop`.
  - `QRWorker` polls the Flask vision server and forwards new QR reads over WebSocket.

- **`rescue_v2.py`** — `MotorClient`, a clean standalone/importable TCP client for the AK45 motor bridge (same port 9000, JSON protocol). Callback-based (`on_feedback`, `on_ack`). This is a focused alternative to the motor handling embedded in `rescue.py`, not wired into it.

- **`app_yolo11.py`** — self-contained Flask vision server (port 5000). Captures an RTSP stream, runs three detect modes (motion / QR / YOLO11 AI using `X7.pt`), exposes MJPEG `/video_feed`, JSON `/status`, `/scan_qr`, and `/ai/*` endpoints, and logs QR/AI hits to CSV. The full HTML UI is an inline `INDEX_HTML` string served at `/`.

- **`ui/`** — browser frontend. `control.html` + `script.js` is the main teleop console (WebSocket to 8765, REST to Flask 5000). `index_3d.html` is a separate Three.js arm visualizer. `script.js` contains a forward/inverse-kinematics model (`ARM_CFG`, `IK_JOINTS`).

- **`models/`** STL meshes for the 3D arm viewer; **`X7.pt`** is the YOLO11 weights file.

## Conventions & gotchas

- **Pi protocol is newline-delimited JSON** over a raw TCP socket on port 9000, both directions. Every command is `{"type": ..., ...}`. When adding a robot command, add it to `RobotController.handle_ws` and emit via `_send_pi`.
- **Servo model is fixed at 8 joints** with parallel `SERVO_NAMES / DEFAULTS / MINS / MAXS` arrays. These constants are **duplicated** between `rescue.py` and `ui/script.js` — keep them in sync. All servo angles must go through `_sv_clamp(idx, v)`.
- **Joint index 5 (J6 Gripper) is special**: `_servo_cmd` translates it into `motor_grip` / `motor_reverse` motor commands instead of a raw `servo_set`.
- **Postures**: `POSTURE_ANGLES` are built-ins; `CUSTOM_POSTURES` (F4–F9) are editable from the browser Settings UI via `save_custom_posture`. F2 `horizontal` and F7 `custom_2` send raw `GOTO` arm commands instead of per-joint angles.
- Network addresses (`PI_IP`, `RTSP_URL`, ports) are hard-coded constants at the top of `rescue.py` and `app_yolo11.py` — these are the first things to edit for a new deployment.

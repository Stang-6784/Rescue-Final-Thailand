# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Teleoperation control software for a search-and-rescue robot. The robot's low-level hardware (Teensy MCU, AK45 BLDC drive motors, 8 arm/flipper servos) lives on a **Raspberry Pi**; this repo is the **Windows operator-side** stack plus a browser UI. Code comments and logs are in Thai.

There is no build system, no test suite, and no `requirements.txt`. Everything is run directly with `python`.

## Repository layout

```
src/                     Python run-time entry points (rescue.py, rescue_v2.py)
ui/                      Browser frontend (HTML/CSS/JS) + ui/models/ (3D STL meshes)
AI Detection/Camera/     YOLO vision server (app_yolo11.py) + X7.pt weights
```

Web sources are split by language — each page links external `.css` and `.js`
(no large inline `<style>`/`<script>` blocks).

## Running the components

These are independent processes; a full session runs them in separate terminals.

```bash
# Operator bridge: HTTP(8766) + WebSocket(8765) + TCP client to the Pi(9000).
# Serves the project root, auto-opens http://localhost:8766/ui/control.html
python src/rescue.py

# Vision server: Flask on :5000 — RTSP camera, motion detection, QR scan, YOLO11 AI
# (loads X7.pt from its own folder — cwd-independent)
python "AI Detection/Camera/app_yolo11.py"

# Standalone AK45 motor test client (host defaults to 192.168.1.111)
python src/rescue_v2.py <PI_IP>
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

- **`src/rescue.py`** — the central operator bridge. Single process running three servers via threads + asyncio:
  - HTTP server (`_http_server`, port 8766) serves the **project root** (so `/ui/…` and `/ui/models/…` resolve) with permissive CORS/iframe headers. `base` is computed as the parent of `src/`.
  - WebSocket server (port 8765) is the browser↔robot command channel. `RobotController.handle_ws` is the giant dispatch switch for every command type (`move_start`, `servo_set`, `posture`, `motor_*`, etc.).
  - TCP client to the Pi (port 9000): `_send_pi` writes newline-delimited JSON; `_pi_recv_thread` reads it back. Connection is auto-reconnecting (`_pi_connect_thread`).
  - State is pushed to all browsers at `STATE_PUSH_HZ` (20Hz) plus on every event; driving keys are re-sent to the Pi at 50Hz from `_send_loop`.
  - `QRWorker` polls the Flask vision server and forwards new QR reads over WebSocket.

- **`src/rescue_v2.py`** — `MotorClient`, a clean standalone/importable TCP client for the AK45 motor bridge (same port 9000, JSON protocol). Callback-based (`on_feedback`, `on_ack`). This is a focused alternative to the motor handling embedded in `rescue.py`, not wired into it.

- **`AI Detection/Camera/app_yolo11.py`** — self-contained Flask vision server (port 5000). Captures an RTSP stream, runs three detect modes (motion / QR / YOLO11 AI using `X7.pt`), exposes MJPEG `/video_feed`, JSON `/status`, `/scan_qr`, and `/ai/*` endpoints, and logs QR/AI hits to CSV. The full HTML UI is an inline `INDEX_HTML` string served at `/`. `AI_MODEL_PATH` resolves `X7.pt` relative to the script dir (`_HERE`).

- **`ui/`** — browser frontend, split by language.
  - `control.html` + `styles.css` + `script.js` — the main teleop console (WebSocket to 8765, REST to Flask 5000). `script.js` holds a forward/inverse-kinematics model (`ARM_CFG`, `IK_JOINTS`) and the duplicated servo constants.
  - `index_3d.html` + `robot3d.css` + `robot3d.js` — the **RESQROBOT** 3D viewer (base + front/back flippers + 4-joint arm) embedded as an `<iframe>` in `control.html` (`#mini-3d-wrap`, inside the scrollable `#left-dock`). It receives `{type:'state', angles, …}` via `postMessage` from `script.js` and also connects to WS :8765 directly for IMU. `SERVO_MAP` maps `state.angles[]` indices → model joints (`j1/j2/j3/j4/front/back`); `MANUAL_HINGES`/`ARM_MOUNT` are the tuned pivot offsets.

- **`ui/models/`** STL meshes for the RESQROBOT viewer (`RESQBASE_*.stl`, `JOINT_1..4.stl`); **`AI Detection/Camera/X7.pt`** is the YOLO11 weights file.

## Conventions & gotchas

- **Pi protocol is newline-delimited JSON** over a raw TCP socket on port 9000, both directions. Every command is `{"type": ..., ...}`. When adding a robot command, add it to `RobotController.handle_ws` and emit via `_send_pi`.
- **Servo model is fixed at 8 joints** with parallel `SERVO_NAMES / SERVO_DEFAULTS / MINS / MAXS` arrays. These constants are **duplicated** between `src/rescue.py` and `ui/script.js` — keep them in sync. All servo angles must go through `_sv_clamp(idx, v)`. The 3D viewer's `SERVO_MAP` (`ui/robot3d.js`) also depends on this index order.
- **Joint index 5 (J6 Gripper) is special**: `_servo_cmd` translates it into `motor_grip` / `motor_reverse` motor commands instead of a raw `servo_set`.
- **Postures**: `POSTURE_ANGLES` are built-ins; `CUSTOM_POSTURES` (F4–F9) are editable from the browser Settings UI via `save_custom_posture`. F2 `horizontal` and F7 `custom_2` send raw `GOTO` arm commands instead of per-joint angles.
- Network addresses (`PI_IP`, `RTSP_URL`, ports) are hard-coded constants at the top of `src/rescue.py` and `AI Detection/Camera/app_yolo11.py` — these are the first things to edit for a new deployment.

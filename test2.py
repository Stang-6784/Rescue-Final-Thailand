# -*- coding: utf-8 -*-
"""
test2.py — Windows-side bridge สำหรับ map saver + scan marker  (prototype)

งานหลักอยู่ที่ Pi แล้ว (Pi code/map_marker_pi.py): pose ด้วย tf2 + เขียน .pgm/.yaml
ไฟล์นี้เป็นแค่ "สะพานบางๆ" ฝั่ง Windows:
  - relay map สด (Pi :8766) → browser  เพื่อให้เห็นแผนที่
  - relay คำสั่ง browser → Pi :8767 (mark / save_map / clear)
  - relay marks (Pi → browser) เพื่อวาดจุด scan
  - รับ save_result (ไฟล์ที่ Pi เขียน) → "เขียนลงดิสก์เครื่อง Windows" จริงๆ
ไม่ต้องคำนวณ pose / เขียน PGM เองเลย (Pi ทำหมด)

ลองเร็วๆ:
    python test2.py
    เปิด http://localhost:8089  → เห็นแผนที่สด, กดปุ่ม SAVE / SCAN QR / SCAN AI

ต่อกับ index.html จริง: เปิด ws://<thisPC>:8767 แล้วส่ง
    {"type":"mark","kind":"qr","text":"<qr text>"}   ตอนกด scan QR
    {"type":"mark","kind":"ai","text":"hazmat ..."}  ตอนกด scan AI
    {"type":"save_map"}                              ตอนกด SAVE
"""

import asyncio
import base64
import datetime
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import websockets

# ═══════════════════════════════════════════════════════════════
#  Config
# ═══════════════════════════════════════════════════════════════
PI_IP           = "192.168.1.67"   # ← ตรงกับ PI_IP ใน rescue.py
MAP_WS_PORT     = 8766             # map_pi.py (OccupancyGrid)
MARKER_WS_PORT  = 8767             # map_marker_pi.py บน Pi (mark/save)
BROWSER_WS_PORT = 8767             # WS ที่ browser มาต่อ (local)
HTTP_PORT       = 8089             # หน้าเทสต์ในตัว

# โฟลเดอร์ปลายทางบนเครื่อง Windows (สร้างให้อัตโนมัติ)
SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "saved_maps")


class State:
    def __init__(self):
        self.latest_map = None
        self.browser_clients = set()
        self.pi_ws = None            # websocket ไป Pi :8767

STATE = State()


def log(msg):
    ts = datetime.datetime.now().strftime("%H:%M:%S")
    print(f"{ts}  {msg}", flush=True)


# ═══════════════════════════════════════════════════════════════
#  Browser ↔ bridge
# ═══════════════════════════════════════════════════════════════
async def browser_broadcast(payload):
    if not STATE.browser_clients:
        return
    data = json.dumps(payload, ensure_ascii=False)
    for ws in list(STATE.browser_clients):
        try:
            await ws.send(data)
        except Exception:
            STATE.browser_clients.discard(ws)


async def send_to_pi(payload):
    ws = STATE.pi_ws
    if ws is None:
        return False
    try:
        await ws.send(json.dumps(payload, ensure_ascii=False))
        return True
    except Exception:
        return False


def write_save_result(files):
    """รับ list ของ {name,b64} จาก Pi → เขียนลงดิสก์ Windows"""
    os.makedirs(SAVE_DIR, exist_ok=True)
    written = []
    for f in files:
        path = os.path.join(SAVE_DIR, f["name"])
        with open(path, "wb") as out:
            out.write(base64.b64decode(f["b64"]))
        written.append(f["name"])
    log(f"[save] เขียนลง {SAVE_DIR} → {', '.join(written)}")
    return written


async def browser_handler(ws):
    STATE.browser_clients.add(ws)
    log(f"[browser] connected (รวม {len(STATE.browser_clients)})")
    try:
        if STATE.latest_map:
            await ws.send(json.dumps({"type": "map", **STATE.latest_map}))
        await send_to_pi({"type": "get"})        # ขอ marks ปัจจุบันจาก Pi
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except Exception:
                continue
            t = msg.get("type", "")
            # คำสั่งทั้งหมด forward ตรงไป Pi
            if t in ("mark", "save_map", "clear_marks", "get"):
                ok = await send_to_pi(msg)
                if not ok:
                    await ws.send(json.dumps(
                        {"type": "saved", "ok": False,
                         "error": f"ต่อ Pi :{MARKER_WS_PORT} ไม่ได้"},
                        ensure_ascii=False))
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        STATE.browser_clients.discard(ws)
        log(f"[browser] disconnected (เหลือ {len(STATE.browser_clients)})")


# ═══════════════════════════════════════════════════════════════
#  Pi :8766  map (OccupancyGrid) → relay ไป browser
# ═══════════════════════════════════════════════════════════════
async def map_client_task():
    url = f"ws://{PI_IP}:{MAP_WS_PORT}"
    while True:
        try:
            log(f"[map] connecting {url}")
            async with websockets.connect(url, max_size=None) as ws:
                log("[map] connected")
                async for raw in ws:
                    try:
                        m = json.loads(raw)
                    except Exception:
                        continue
                    if m.get("type") != "map":
                        continue
                    STATE.latest_map = {
                        "w": m["w"], "h": m["h"], "res": m["res"],
                        "ox": m["ox"], "oy": m["oy"], "data": m["data"],
                    }
                    await browser_broadcast({"type": "map", **STATE.latest_map})
        except Exception as e:
            log(f"[map] disconnected: {e} — retry 2s")
            await asyncio.sleep(2)


# ═══════════════════════════════════════════════════════════════
#  Pi :8767  marker node → relay marks/save_result ไป browser
# ═══════════════════════════════════════════════════════════════
async def marker_client_task():
    url = f"ws://{PI_IP}:{MARKER_WS_PORT}"
    while True:
        try:
            log(f"[marker] connecting {url}")
            async with websockets.connect(url, max_size=None) as ws:
                STATE.pi_ws = ws
                log("[marker] connected")
                await ws.send(json.dumps({"type": "get"}))
                async for raw in ws:
                    try:
                        msg = json.loads(raw)
                    except Exception:
                        continue
                    t = msg.get("type", "")
                    if t == "marks":
                        await browser_broadcast(msg)
                    elif t == "save_result":
                        if msg.get("ok"):
                            names = write_save_result(msg.get("files", []))
                            await browser_broadcast({"type": "saved", "ok": True,
                                                     "dir": SAVE_DIR, "files": names})
                        else:
                            await browser_broadcast({"type": "saved", "ok": False,
                                                     "error": msg.get("error", "save failed")})
        except Exception as e:
            STATE.pi_ws = None
            log(f"[marker] disconnected: {e} — retry 2s")
            await asyncio.sleep(2)
        finally:
            STATE.pi_ws = None


# ═══════════════════════════════════════════════════════════════
#  หน้าเทสต์ในตัว (HTTP :8089)
# ═══════════════════════════════════════════════════════════════
TEST_PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>test2 — Map Saver + Marks</title>
<style>
 body{margin:0;background:#04060a;color:#cfe;font-family:Consolas,monospace}
 #bar{padding:8px;display:flex;gap:8px;align-items:center;background:#0a121e;
      border-bottom:1px solid #1d2a3a}
 button{font-family:inherit;font-weight:700;padding:6px 14px;border-radius:6px;
        border:1px solid #00d4ff;background:#061520;color:#00d4ff;cursor:pointer}
 button.qr{border-color:#7CFC00;color:#7CFC00}
 button.ai{border-color:#ff5d5d;color:#ff5d5d}
 #stat{margin-left:auto;font-size:12px;color:#789}
 #wrap{display:flex;height:calc(100vh - 47px)}
 #c{flex:1;background:#04060a}
 #side{width:280px;border-left:1px solid #1d2a3a;overflow:auto;padding:8px;font-size:12px}
 .mk{padding:4px 6px;border-bottom:1px solid #16202e}
 .mk b{color:#fff} .mk.qr b{color:#7CFC00} .mk.ai b{color:#ff5d5d}
 #note{font-size:11px;color:#567;padding:4px 8px}
</style></head><body>
<div id="bar">
  <button onclick="save()">&#128190; SAVE (.pgm/.yaml)</button>
  <button class="qr" onclick="scan('qr')">SCAN QR</button>
  <button class="ai" onclick="scan('ai')">SCAN AI (hazmat)</button>
  <button onclick="ws.send(JSON.stringify({type:'clear_marks'}))">CLEAR MARKS</button>
  <span id="stat">connecting...</span>
</div>
<div id="wrap"><canvas id="c"></canvas><div id="side"><b>MARKS</b><div id="marks"></div></div></div>
<div id="note"></div>
<script>
let ws, map=null, marks=[];
const c=document.getElementById('c'), ctx=c.getContext('2d');
const stat=document.getElementById('stat'), note=document.getElementById('note');
function connect(){
  ws=new WebSocket('ws://'+location.hostname+':%PORT%');
  ws.onopen=()=>{stat.textContent='LIVE';stat.style.color='#7CFC00';ws.send('{"type":"get"}');};
  ws.onclose=()=>{stat.textContent='offline';stat.style.color='#ff5d5d';setTimeout(connect,1500);};
  ws.onmessage=(e)=>{const m=JSON.parse(e.data);
    if(m.type==='map'){map=m;draw();}
    else if(m.type==='marks'){marks=m.marks;draw();renderList();}
    else if(m.type==='saved'){
      if(m.ok) note.innerHTML='&#9989; saved → '+m.dir+'  ['+m.files.join(', ')+']';
      else note.innerHTML='&#10060; '+m.error;
    }
  };
}
function scan(kind){
  const text = kind==='qr' ? prompt('QR text (จำลอง):','QR-DEMO-001')
                           : prompt('AI/hazmat label (จำลอง):','hazmat: corrosive');
  if(text===null) return;
  ws.send(JSON.stringify({type:'mark',kind:kind,text:text}));
}
function save(){ note.textContent='saving...'; ws.send('{"type":"save_map"}'); }
function draw(){
  const W=c.clientWidth,H=c.clientHeight; c.width=W;c.height=H;
  ctx.fillStyle='#04060a';ctx.fillRect(0,0,W,H);
  if(!map) return;
  const w=map.w,h=map.h,off=document.createElement('canvas');off.width=w;off.height=h;
  const o=off.getContext('2d'),img=o.createImageData(w,h);
  for(let y=0;y<h;y++){const src=(h-1-y)*w;for(let x=0;x<w;x++){
    const v=map.data[src+x],di=(y*w+x)*4;let r,g,b;
    if(v<0){r=38;g=44;b=56;}else if(v>=50){r=0;g=212;b=255;}else{r=16;g=24;b=40;}
    img.data[di]=r;img.data[di+1]=g;img.data[di+2]=b;img.data[di+3]=255;}}
  o.putImageData(img,0,0);
  const s=Math.min(W/w,H/h),dw=w*s,dh=h*s,ox=(W-dw)/2,oy=(H-dh)/2;
  ctx.imageSmoothingEnabled=false;ctx.drawImage(off,0,0,w,h,ox,oy,dw,dh);
  marks.forEach(mk=>{
    if(!mk.pose_ok) return;
    const col=(mk.x-map.ox)/map.res, rowB=(mk.y-map.oy)/map.res;
    const px=ox+col*s, py=oy+(h-1-rowB)*s;
    ctx.beginPath();ctx.arc(px,py,6,0,7);
    ctx.fillStyle=mk.kind==='qr'?'#7CFC00':'#ff5d5d';ctx.fill();
    ctx.strokeStyle='#000';ctx.stroke();
    ctx.fillStyle='#fff';ctx.font='10px monospace';
    ctx.fillText((mk.kind==='qr'?'QR#':'AI#')+mk.id,px+8,py+3);
  });
}
function renderList(){
  document.getElementById('marks').innerHTML = marks.map(mk=>
    '<div class="mk '+mk.kind+'"><b>'+(mk.kind==='qr'?'QR':'AI')+' #'+mk.id+'</b> '
    +(mk.text||'')+'<br><span style="color:#678">'
    +(mk.pose_ok?('('+mk.x.toFixed(2)+', '+mk.y.toFixed(2)+') m'):'pose unknown')
    +' · '+mk.t+'</span></div>').join('') || '<div style="color:#567">ยังไม่มี</div>';
}
window.addEventListener('resize',draw);
connect();
</script></body></html>"""


def http_server_thread():
    page = TEST_PAGE.replace("%PORT%", str(BROWSER_WS_PORT)).encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)

        def log_message(self, *a):
            pass

    ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler).serve_forever()


# ═══════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════
async def amain():
    async with websockets.serve(browser_handler, "0.0.0.0", BROWSER_WS_PORT,
                                max_size=None, ping_interval=20):
        log(f"[ws] browser WS ฟังที่ ws://0.0.0.0:{BROWSER_WS_PORT}")
        await asyncio.gather(map_client_task(), marker_client_task())


def main():
    import sys
    if sys.platform == "win32":
        try: sys.stdout.reconfigure(encoding="utf-8")
        except Exception: pass
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    threading.Thread(target=http_server_thread, daemon=True).start()

    print("=" * 64)
    print("  test2.py — Map Saver + Scan Marker (Windows bridge)")
    print(f"  Map src   → ws://{PI_IP}:{MAP_WS_PORT}")
    print(f"  Marker    → ws://{PI_IP}:{MARKER_WS_PORT}  (map_marker_pi.py)")
    print(f"  Browser   → ws://localhost:{BROWSER_WS_PORT}")
    print(f"  Test page → http://localhost:{HTTP_PORT}")
    print(f"  Save dir  → {SAVE_DIR}")
    print("=" * 64)

    try:
        asyncio.run(amain())
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()

// ============================================================================
//  map_panel.js  —  วาด SLAM map (จาก Pi) ลงช่อง LIDAR MAP (#lidarCanvas)
//  ของหน้า control-center เดิม โดยเปิด WebSocket เส้นที่ 2 ตรงไปที่ Pi (:8766)
//  แยกจากระบบควบคุมหุ่น (ws :8765) — ไม่ต้องแก้ rescue.py
//
//  วิธีใช้: วางไฟล์นี้ไว้ข้าง index.html แล้วเพิ่มบรรทัดนี้ก่อน </body>
//      <script src="map_panel.js"></script>
//  (วางหลัง <script src="script_index.js"></script> เพื่อให้ใช้ #piAddr ได้)
// ============================================================================
(function () {
  const canvas = document.getElementById('lidarCanvas');
  if (!canvas) { console.warn('[map] ไม่พบ #lidarCanvas'); return; }
  const ctx = canvas.getContext('2d');
  const statusEl = document.getElementById('lidarStatus');
  const rangeTag = document.getElementById('lidar-range-tag');
  const off = document.createElement('canvas');   // offscreen: map ความละเอียดจริง

  let ws = null, retryT = null, staleT = null;

  // map_ws_server.py รันบน Pi ที่พอร์ต 8766 (คนละเส้นกับ control WS 8765)
  // ดึง host จากช่อง PI (#piAddr) เป็นหลัก, สำรองด้วย wsAddr / hostname
  function wsHost() {
    const pi  = document.getElementById('piAddr');
    const wsA = document.getElementById('wsAddr');
    let host = (pi && pi.textContent ? pi.textContent.trim() : '')
            || (wsA && wsA.value ? wsA.value.trim() : '')
            || location.hostname || 'localhost';
    host = host.split(':')[0];          // ตัดพอร์ตเดิมทิ้ง เอาเฉพาะ IP/host
    return `${host}:8766`;              // ⟵ ต่อ map_ws_server.py
  }


  function setLive(on) {
    if (!statusEl) return;
    statusEl.textContent = on ? 'LIVE' : 'NO DATA';
    statusEl.className = on ? 'lidar-live' : 'lidar-stale';
  }

  function connect() {
    const url = `ws://${wsHost()}`;
    try { ws = new WebSocket(url); }
    catch (e) { return retry(); }

    ws.onmessage = (ev) => {
      let m; try { m = JSON.parse(ev.data); } catch (e) { return; }
      if (m.type !== 'map') return;
      drawMap(m);
      setLive(true);
      clearTimeout(staleT);
      staleT = setTimeout(() => setLive(false), 4000);  // ไม่มี map ใหม่ใน 4s = stale
    };
    ws.onclose = () => { setLive(false); retry(); };
    ws.onerror = () => { try { ws.close(); } catch (e) {} };
  }

  function retry() { clearTimeout(retryT); retryT = setTimeout(connect, 2000); }

  function drawMap(m) {
    const w = m.w, h = m.h;
    if (!w || !h) return;

    // 1) วาด map ลง offscreen ที่ความละเอียดจริง (1 cell = 1 px)
    off.width = w; off.height = h;
    const octx = off.getContext('2d');
    const img = octx.createImageData(w, h);
    for (let y = 0; y < h; y++) {
      const src = (h - 1 - y) * w;                 // OccupancyGrid เริ่มมุมล่างซ้าย -> พลิกแนวตั้ง
      for (let x = 0; x < w; x++) {
        const v = m.data[src + x];
        const di = (y * w + x) * 4;
        let r, g, b;
        if (v < 0)        { r = 38;  g = 44;  b = 56;  }   // unknown -> เทาเข้ม
        else if (v >= 50) { r = 0;   g = 212; b = 255; }   // obstacle -> ฟ้า accent
        else              { r = 16;  g = 24;  b = 40;  }   // free -> น้ำเงินเข้ม
        img.data[di] = r; img.data[di + 1] = g; img.data[di + 2] = b; img.data[di + 3] = 255;
      }
    }
    octx.putImageData(img, 0, 0);

    // 2) ขยายลง canvas จริงแบบรักษาอัตราส่วน + จัดกึ่งกลาง
    const cw = canvas.clientWidth  || canvas.width  || 240;
    const ch = canvas.clientHeight || canvas.height || 160;
    canvas.width = cw; canvas.height = ch;
    ctx.fillStyle = '#04060a';
    ctx.fillRect(0, 0, cw, ch);
    const s = Math.min(cw / w, ch / h);
    const dw = w * s, dh = h * s, ox = (cw - dw) / 2, oy = (ch - dh) / 2;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(off, 0, 0, w, h, ox, oy, dw, dh);

    if (rangeTag) rangeTag.textContent = `${(w * m.res).toFixed(1)}×${(h * m.res).toFixed(1)} m`;
  }

  connect();
})();

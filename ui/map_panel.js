// ============================================================================
//  map_panel.js  —  วาด SLAM map (จาก Pi) ลงช่อง LIDAR MAP (#lidarCanvas)
//
//  ที่มาของ map: บน Pi รัน map_ws_server.py (map_pi.py) เป็น WebSocket server
//  ที่พอร์ต 8766 ส่ง {type:'map',...} ตรงๆ (คนละเส้นกับ control WS 8765)
//  ไฟล์นี้จึงเปิด WS เส้นที่ 2 ตรงไปที่ Pi:8766 — ไม่เกี่ยวกับ rescue.py
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
  let lastMap = null;   // map ล่าสุดที่วาด — ใช้ตอน save CSV

  // Pi:8766 = map_ws_server.py — ดึง IP Pi จากช่อง PI (#piAddr) เป็นหลัก
  // (#piAddr ต้องตรงกับ PI_IP ใน rescue.py ไม่งั้นต่อ map ไม่ติด)
  function wsHost() {
    const pi = document.getElementById('piAddr');
    let host = (pi && pi.textContent ? pi.textContent.trim() : '')
            || location.hostname || 'localhost';
    host = host.split(':')[0];          // เอาเฉพาะ IP/host ตัดพอร์ตทิ้ง
    return `${host}:8766`;
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
    lastMap = m;   // เก็บไว้ให้ saveMapCSV ใช้

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

  // ── Save map → .csv ──────────────────────────────────────────
  // ส่งออกเป็นตาราง h แถว × w คอลัมน์ (ค่า -1=unknown, 0=ว่าง, 100=สิ่งกีดขวาง)
  // เรียงตามภาพที่เห็น (แถวบนสุดก่อน) — เปิดใน Excel ได้เลย
  window.saveMapCSV = function () {
    if (!lastMap || !lastMap.data) {
      alert('ยังไม่มีข้อมูล map ให้บันทึก'); return;
    }
    const m = lastMap, w = m.w, h = m.h, d = m.data;
    const lines = [];
    for (let y = 0; y < h; y++) {
      const src = (h - 1 - y) * w;        // grid เริ่มมุมล่างซ้าย → พลิกให้แถวบนอยู่ก่อน
      const row = new Array(w);
      for (let x = 0; x < w; x++) row[x] = d[src + x];
      lines.push(row.join(','));
    }
    const csv = lines.join('\r\n');
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    const ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    a.href = url;
    a.download = `map_${w}x${h}_res${m.res}_${ts}.csv`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
    if (window.addLog) addLog(`Map saved: ${w}x${h} → CSV`, 'ok', 'detect');
  };

  // ── Reset map ────────────────────────────────────────────────
  // ล้างภาพฝั่ง browser แล้ว reconnect เพื่อดึง map สดจาก Pi ใหม่
  // หมายเหตุ: นี่ล้าง "การแสดงผล" เท่านั้น — ถ้าต้องการให้ SLAM (Cartographer)
  // เริ่มสร้างแผนที่ใหม่จริงๆ ต้องสั่ง reset ฝั่ง Pi (map_pi.py ยังไม่รองรับ)
  window.resetMap = function () {
    lastMap = null;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#04060a';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (rangeTag) rangeTag.textContent = '— m';
    setLive(false);
    // ส่งคำขอ reset เผื่อ Pi รองรับในอนาคต (ตอนนี้ map_pi.py ไม่อ่าน → ถูกเพิกเฉย)
    try { if (ws && ws.readyState === 1) ws.send(JSON.stringify({ type: 'reset_map' })); } catch (e) {}
    // reconnect เพื่อดึง map ปัจจุบันจาก Pi ใหม่ (Pi ส่ง last_map ให้ทันทีตอนต่อ)
    try { if (ws) ws.close(); } catch (e) {}
    if (window.addLog) addLog('Map view reset', 'warn', 'detect');
  };

  connect();
})();

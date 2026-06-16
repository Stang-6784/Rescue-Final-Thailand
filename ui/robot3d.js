// ════════════════════════════════════════════════════════════════════════════
//  robot3d.js — RESQROBOT 3D viewer (ฝังใน control.html ผ่าน index_3d.html)
//
//  รวมโค้ดจาก 2 ส่วน:
//   • โมเดลใหม่ "RESQROBOT" (ฐาน + ตีนตะขาบหน้า/หลัง + แขนกล 4 ข้อ) — แทนโมเดลเดิม
//   • การเชื่อมต่อจาก index_3d.html เดิม : WebSocket :8765, postMessage จาก
//     control.html (iframe), การหมุนตาม IMU, และการ LERP ให้ขยับนุ่มนวล
//
//  ขับเคลื่อนข้อต่อตามองศาของแต่ละ servo ที่ส่งมาจาก control.html
// ════════════════════════════════════════════════════════════════════════════
import * as THREE from 'three';
import { STLLoader }     from 'three/addons/loaders/STLLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const EMBEDDED = (window.self !== window.top);   // อยู่ใน iframe ของ control.html?
document.body.classList.toggle('embedded', EMBEDDED);

// ── นิยามชิ้นส่วน ─────────────────────────────────────────────────────────────
//   parent: 'root'  → หมุนใต้ฐาน/โลก
//           'mount' → หมุนใต้กลุ่ม arm-mount (ตำแหน่งติดตั้งแขน)
//           '<key>' → หมุนใต้ mover ตัวนั้น (kinematic chain)
const MODEL_DIR = './models/';
const BASE = { url: MODEL_DIR + 'RESQBASE_BASE.stl', color: 0x6b7c93 };

const MOVERS = [
    { key:'front', label:'FRONT flipper', url: MODEL_DIR + 'RESQBASE_FRONT.stl', color:0xa855f7, group:'flipper', parent:'root',  axis:'x', dir:-1, min:-90, max:90  },
    { key:'back',  label:'BACK flipper',  url: MODEL_DIR + 'RESQBASE_BACK.stl',  color:0xa855f7, group:'flipper', parent:'root',  axis:'x', dir:+1, min:-90, max:90  },
    { key:'j1',    label:'JOINT_1',       url: MODEL_DIR + 'JOINT_1.stl', color:0x3b82f6, group:'arm', parent:'mount', axis:'x', dir:+1, min:-180, max:180 },
    { key:'j2',    label:'JOINT_2',       url: MODEL_DIR + 'JOINT_2.stl', color:0x06b6d4, group:'arm', parent:'j1',    axis:'x', dir:+1, min:-180, max:180 },
    { key:'j3',    label:'JOINT_3',       url: MODEL_DIR + 'JOINT_3.stl', color:0xf59e0b, group:'arm', parent:'j2',    axis:'x', dir:+1, min:-180, max:180 },
    { key:'j4',    label:'JOINT_4',       url: MODEL_DIR + 'JOINT_4.stl', color:0xef4444, group:'arm', parent:'j3',    axis:'x', dir:+1, min:-180, max:180 },
];

// จุดหมุน + ตำแหน่งติดตั้งแขน — ค่าที่จูนไว้แล้วจาก simulator ต้นฉบับ
const MANUAL_HINGES = {
    front: { x: 13, y: 15, z: 72  },
    back:  { x: 13, y: 15, z: -57 },
    j1:    { x: 12, y: -40, z: -341 },
    j2:    { x: 15, y: 22,  z: -302 },
    j3:    { x: 15, y: 54,  z: -480 },
    j4:    { x: 15, y: 60,  z: -321 },
};
const ARM_MOUNT = { x: 0, y: 100, z: 400, rx: 0, ry: 0, rz: 0, s: 1 };

// ── การ map servo (control.html) → ข้อต่อในโมเดล ─────────────────────────────
//  state.angles[] :  [0]J1 Shoulder [1]J2 Elbow [2]J3 Extend [3]J4 Wrist
//                    [4]J5 Tool     [5]J6 Gripper [6]Flip-F   [7]Flip-R
//  neutral = ค่า servo ที่ทำให้ข้อต่ออยู่ตำแหน่ง 0°  ·  scale = อัตราขยายการหมุน
const SERVO_MAP = {
    0: { key:'j1',    neutral: 50,  scale: 1.0 },  // J1 Shoulder
    1: { key:'j2',    neutral: 130, scale: 1.0 },  // J2 Elbow
    3: { key:'j3',    neutral: 90,  scale: 1.0 },  // J4 Wrist
    4: { key:'j4',    neutral: 90,  scale: 1.0 },  // J5 Tool
    6: { key:'front', neutral: 100, scale: 1.0 },  // Flip-F
    7: { key:'back',  neutral: 100, scale: 1.0 },  // Flip-R
};

// ── Renderer / Scene ─────────────────────────────────────────────────────────
const canvas   = document.getElementById('three-canvas');
const viewport = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0f16);
scene.add(new THREE.HemisphereLight(0x8eb1c7, 0x2d3748, 0.75));
const sun = new THREE.DirectionalLight(0xfff4e0, 1.4);
sun.position.set(500, 800, 600); sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048);
const sc = 1000; Object.assign(sun.shadow.camera, { left:-sc, right:sc, top:sc, bottom:-sc, near:1, far:5000 });
scene.add(sun);
const rim = new THREE.DirectionalLight(0x4466aa, 0.5); rim.position.set(-400, 200, -400); scene.add(rim);
scene.add(new THREE.GridHelper(2000, 50, 0x1f2937, 0x141b26));

const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 40000);
camera.position.set(700, 500, 700);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true; controls.dampingFactor = 0.06;
controls.minDistance = 30; controls.maxDistance = 12000;

// tiltGroup = หมุนทั้งตัวตาม IMU,  rootGroup = ตัวหุ่น,  armMount = กลุ่มแขน
const tiltGroup = new THREE.Group(); scene.add(tiltGroup);
const rootGroup = new THREE.Group(); tiltGroup.add(rootGroup);
const armMount  = new THREE.Group(); rootGroup.add(armMount);
let baseMesh = null;

const state = {};   // key → { def, pivot, hinge, cur, tgt, axis, dir, mesh }
MOVERS.forEach(m => state[m.key] = {
    def: m, pivot: new THREE.Group(), hinge: new THREE.Vector3(),
    cur: 0, tgt: 0, axis: m.axis, dir: m.dir, mesh: null,
});
let loaded = 0;
const TOTAL = MOVERS.length + 1;   // +1 สำหรับฐาน

// ── การโหลด STL ──────────────────────────────────────────────────────────────
const loader = new STLLoader();
const makeMat = (color) => new THREE.MeshPhysicalMaterial({ color, metalness: 0.4, roughness: 0.5 });
const loadDetail = document.getElementById('load-detail');

loader.load(BASE.url,
    (geo) => { geo.computeVertexNormals(); baseMesh = new THREE.Mesh(geo, makeMat(BASE.color));
        baseMesh.castShadow = baseMesh.receiveShadow = true;
        if (loadDetail) loadDetail.textContent = 'BASE ✓';
        loaded++; tryAssemble(); },
    null,
    () => { baseMesh = new THREE.Mesh(new THREE.BoxGeometry(300, 100, 500), makeMat(BASE.color));
        if (loadDetail) loadDetail.textContent = BASE.url + ' — ต้องรันผ่าน HTTP server';
        loaded++; tryAssemble(); });

MOVERS.forEach(m => {
    loader.load(m.url,
        (geo) => { geo.computeVertexNormals(); const mesh = new THREE.Mesh(geo, makeMat(m.color));
            mesh.castShadow = mesh.receiveShadow = true; state[m.key].mesh = mesh;
            if (loadDetail) loadDetail.textContent = m.label + ' ✓';
            loaded++; tryAssemble(); },
        null,
        () => { state[m.key].mesh = new THREE.Mesh(new THREE.BoxGeometry(80, 40, 120), makeMat(m.color));
            loaded++; tryAssemble(); });
});

// ── ประกอบหุ่น ───────────────────────────────────────────────────────────────
function tryAssemble() {
    if (loaded < TOTAL) return;

    baseMesh.geometry.computeBoundingBox();
    autoHinges();
    MOVERS.forEach(m => { const h = MANUAL_HINGES[m.key]; if (h) state[m.key].hinge.set(h.x, h.y, h.z); });

    rootGroup.add(baseMesh);
    MOVERS.forEach(m => { state[m.key].mesh.geometry.computeBoundingBox(); });

    MOVERS.forEach(m => {
        const s = state[m.key];
        s.pivot.add(s.mesh);
        parentOf(m).add(s.pivot);
    });

    // วางกลุ่มแขนบนฐานตามค่า ARM_MOUNT
    armMount.position.set(ARM_MOUNT.x, ARM_MOUNT.y, ARM_MOUNT.z);
    armMount.rotation.set(THREE.MathUtils.degToRad(ARM_MOUNT.rx),
                          THREE.MathUtils.degToRad(ARM_MOUNT.ry),
                          THREE.MathUtils.degToRad(ARM_MOUNT.rz));
    armMount.scale.setScalar(ARM_MOUNT.s);

    rebuildChain();

    // วางหุ่นให้นั่งบนกริด
    const bb = new THREE.Box3().setFromObject(rootGroup);
    rootGroup.position.y = -bb.min.y;

    fitCamera();
    MOVERS.forEach(m => setMoverAngle(m.key, 0));

    const overlay = document.getElementById('loading-overlay');
    if (overlay) overlay.style.display = 'none';
}

// Object3D ที่ pivot ของ mover จะไปเกาะ
function parentOf(m) {
    if (m.parent === 'root')  return rootGroup;
    if (m.parent === 'mount') return armMount;
    return state[m.parent].pivot;
}
function parentHinge(m) {
    if (m.parent === 'root' || m.parent === 'mount') return new THREE.Vector3(0, 0, 0);
    return state[m.parent].hinge;
}

// เดาจุดหมุนอัตโนมัติ (ใช้เมื่อไม่มีค่า MANUAL_HINGES)
function autoHinges() {
    const bbOf = (mesh) => { mesh.geometry.computeBoundingBox(); return mesh.geometry.boundingBox.clone(); };
    const centre = (b) => new THREE.Vector3((b.min.x+b.max.x)/2, (b.min.y+b.max.y)/2, (b.min.z+b.max.z)/2);
    const baseBB = bbOf(baseMesh);
    const overlapCentre = (a, b) => {
        const lo = new THREE.Vector3(Math.max(a.min.x,b.min.x), Math.max(a.min.y,b.min.y), Math.max(a.min.z,b.min.z));
        const hi = new THREE.Vector3(Math.min(a.max.x,b.max.x), Math.min(a.max.y,b.max.y), Math.min(a.max.z,b.max.z));
        if (lo.x<=hi.x && lo.y<=hi.y && lo.z<=hi.z) return new THREE.Vector3((lo.x+hi.x)/2,(lo.y+hi.y)/2,(lo.z+hi.z)/2);
        return centre(a).add(centre(b)).multiplyScalar(0.5);
    };
    MOVERS.forEach(m => {
        const partBB = bbOf(state[m.key].mesh);
        if (m.group === 'flipper' || m.parent === 'mount') state[m.key].hinge.copy(overlapCentre(partBB, baseBB));
        else state[m.key].hinge.copy(overlapCentre(partBB, bbOf(state[m.parent].mesh)));
    });
}

function rebuildChain() {
    MOVERS.forEach(m => {
        const s = state[m.key];
        s.pivot.position.copy(s.hinge).sub(parentHinge(m));
        s.mesh.position.copy(s.hinge).multiplyScalar(-1);
    });
}

function fitCamera() {
    const sph = new THREE.Sphere();
    new THREE.Box3().setFromObject(rootGroup).getBoundingSphere(sph);
    const r = sph.radius || 300;
    camera.position.set(sph.center.x + r*2.0, sph.center.y + r*1.3, sph.center.z + r*2.0);
    controls.target.copy(sph.center); controls.update();
}

// ── การหมุนข้อต่อ ────────────────────────────────────────────────────────────
//  ตั้ง "เป้าหมาย" (tgt) ของข้อต่อ แล้วให้ลูป LERP ค่อย ๆ ขยับ cur เข้าหา
function setMoverTarget(key, deg) {
    const s = state[key]; if (!s) return;
    s.tgt = Math.max(s.def.min, Math.min(s.def.max, deg));
}
// ตั้งองศาทันที (ไม่ LERP) — ใช้ตอน assemble
function setMoverAngle(key, deg) {
    const s = state[key]; if (!s) return;
    s.cur = s.tgt = Math.max(s.def.min, Math.min(s.def.max, deg));
    applyMoverRotation(s);
}
function applyMoverRotation(s) {
    s.pivot.rotation.set(0, 0, 0);
    s.pivot.rotation[s.axis] = THREE.MathUtils.degToRad(s.dir * s.cur);
}

// ── รับค่า servo จาก control.html / WebSocket ────────────────────────────────
function applyAngles(angles) {
    if (!Array.isArray(angles)) return;
    for (const idx in SERVO_MAP) {
        const v = angles[idx];
        if (typeof v !== 'number') continue;
        const map = SERVO_MAP[idx];
        setMoverTarget(map.key, (v - map.neutral) * map.scale);
    }
}

function applyIMU(roll, pitch, yaw) {
    setText('vX', (+roll).toFixed(1));
    setText('vY', (+pitch).toFixed(1));
    setText('vZ', (+yaw).toFixed(1));
    tiltGroup.quaternion.setFromEuler(new THREE.Euler(
        THREE.MathUtils.degToRad(roll),
        THREE.MathUtils.degToRad(yaw),
        THREE.MathUtils.degToRad(-pitch), 'XYZ'));
}

const setText = (id, t) => { const el = document.getElementById(id); if (el) el.textContent = t; };

// ── WebSocket (เชื่อม rescue.py :8765 — ใช้รับ IMU + state ตอนเปิดเดี่ยว ๆ) ───
const wsBadge = document.getElementById('wsBadge');
let ws = null, retryTimer = null, retryDelay = 1000;

function wsState(cls) { if (wsBadge) wsBadge.className = cls; }

function connectWS() {
    const host = (location.hostname || 'localhost') + ':8765';
    const url = `ws://${host}`;
    wsState('ws-retry');
    try { ws = new WebSocket(url); } catch (_) { scheduleRetry(); return; }
    ws.onopen = () => { retryDelay = 1000; wsState('ws-on'); };
    ws.onmessage = (evt) => {
        try {
            const msg = JSON.parse(evt.data);
            if (msg.type === 'state') {
                if (Array.isArray(msg.angles)) applyAngles(msg.angles);
                if (msg.imu) applyIMU(msg.imu.roll, msg.imu.pitch, msg.imu.yaw);
            } else if (msg.type === 'imu') {
                applyIMU(msg.roll, msg.pitch, msg.yaw);
            }
        } catch (_) {}
    };
    ws.onerror = () => wsState('ws-err');
    ws.onclose = () => { ws = null; wsState('ws-off'); scheduleRetry(); };
}
function scheduleRetry() {
    clearTimeout(retryTimer);
    retryTimer = setTimeout(() => { retryDelay = Math.min(10000, retryDelay * 1.5); connectWS(); }, retryDelay);
}

// ── postMessage จาก control.html (iframe) — เส้นทางหลักตอนฝังอยู่ ─────────────
window.addEventListener('message', (evt) => {
    const msg = evt.data;
    if (!msg || msg.type !== 'state') return;
    if (Array.isArray(msg.angles)) applyAngles(msg.angles);
    if (msg.imu) applyIMU(msg.imu.roll, msg.imu.pitch, msg.imu.yaw);
});

// ── Resize ตามขนาดกรอบ (#viewport) — ใช้ ResizeObserver ให้ฟิตทุกขนาด ────────
function resize() {
    const w = viewport.clientWidth, h = viewport.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport);
window.addEventListener('resize', resize);
resize();

// ── ลูปเรนเดอร์ + LERP ให้ข้อต่อขยับนุ่มนวล ──────────────────────────────────
const LERP = 0.18;
(function animate() {
    requestAnimationFrame(animate);
    MOVERS.forEach(m => {
        const s = state[m.key];
        if (Math.abs(s.tgt - s.cur) > 0.01) { s.cur += (s.tgt - s.cur) * LERP; applyMoverRotation(s); }
    });
    controls.update();
    renderer.render(scene, camera);
})();

// ── API สำหรับ debug / ใช้งานภายนอก ──────────────────────────────────────────
window.RESQROBOT = {
    set: (key, deg) => setMoverTarget(key, deg),
    applyAngles,
    applyIMU,
    getState: () => MOVERS.reduce((o, m) => (o[m.key] = state[m.key].cur, o), {}),
    reset: () => MOVERS.forEach(m => setMoverTarget(m.key, 0)),
};

connectWS();

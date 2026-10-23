import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { CSS2DRenderer, CSS2DObject } from "three/addons/renderers/CSS2DRenderer.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineSegments2 } from "three/addons/lines/LineSegments2.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineSegmentsGeometry } from "three/addons/lines/LineSegmentsGeometry.js";
import { SparkRenderer, SplatMesh, SplatEdit, SplatEditSdf, SplatEditSdfType, SplatEditRgbaBlendMode } from "@sparkjsdev/spark";
import { loadCad } from "./cad.js";
import { loadIfc, styleModel, floorLevels, markerGroups, colorMarkers } from "./bim.js";
import { parsePlyHeader, isGaussianPly, readPointCloud, estimateSpacing } from "./ply.js";
import { sampleSplats, detectFloor, floorTransform, computeCore, filterToBox } from "./floor.js";
import { snapIndex, snapEndpoint, snapCorner } from "./snap.js";
import { drawingField, bimField, sliceScan, refine, globalSearch, poseMatrix } from "./autoalign.js";
import { detectMarkers, matchMarkers } from "./markers.js";
import { addMarkersToIfc, zipOne } from "./ifcexport.js";
import { extractFaces, facesToWalls, buildScanIfc, buildMeshIfc } from "./scan2ifc.js";
import { voxelBoxes } from "./voxel.js";
import { MESH_EXT, loadMeshModel } from "./mesh.js";
import { newPersistId, persistFile, persistMeta, persistDelete, persistLoadAll } from "./persist.js";

const $ = (id) => document.getElementById(id);
const UP = new THREE.Vector3(0, 1, 0);
const DEG = Math.PI / 180;
const LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
let uid = 1;

// =====================================================================
// 1. Scene
// =====================================================================
const container = $("canvas-container");
const canvas = $("c");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.localClippingEnabled = true;
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1d24);
const camera = new THREE.PerspectiveCamera(50, 1, 0.02, 5000);
camera.position.set(8, 7, 10);
const spark = new SparkRenderer({ renderer });
// fixed draw order between the scan and BIM models: three.js otherwise sorts the transparent layers by
// distance, so from some angles the model was drawn last and covered the scan
// "scan in front": models are drawn first and always write depth (pushed back slightly so surfaces that
// coincide go to the scan), then the scan; a transparent model still shows wherever it is nearer than the
// scanned surface. "BIM in front": the scan first, then the model blended over it.
// (Splats never write depth: their soft edges would cut hard arcs into each other.)
let frontScan = true;
spark.renderOrder = 10;
function applyFront(r) {
  if (r.type !== "bim") return;
  for (const m of r.materials) {
    m.depthWrite = frontScan || m.opacity >= 0.99;
    m.polygonOffset = true; // faces sit a hair behind their outline lines (and behind coincident scan surfaces)
    m.polygonOffsetFactor = 1;
    m.polygonOffsetUnits = 4;
    m.needsUpdate = true;
  }
}
scene.add(spark);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.1;
controls.screenSpacePanning = true;

scene.add(new THREE.AmbientLight(0xffffff, 0.8));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(10, 20, 10);
scene.add(sun);

const helpers = new THREE.Group();
scene.add(helpers);

const labelRenderer = new CSS2DRenderer();
labelRenderer.domElement.style.cssText = "position:absolute;inset:0;pointer-events:none";
container.appendChild(labelRenderer.domElement);

function resize() {
  const w = container.clientWidth, h = container.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  labelRenderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  for (const m of fatMaterials) m.resolution.set(w, h);
}
// thick screen-space lines (WebGL lines are always 1 px)
const fatMaterials = new Set();
function fatMaterial(color, width, opacity = 1) {
  const m = new LineMaterial({ color, linewidth: width, transparent: true, opacity, depthTest: false });
  m.resolution.set(container.clientWidth || 1, container.clientHeight || 1);
  fatMaterials.add(m);
  return m;
}
function fatLine(a, b, color, width) {
  const g = new LineGeometry();
  g.setPositions([a.x, a.y, a.z, b.x, b.y, b.z]);
  const l = new Line2(g, fatMaterial(color, width));
  l.computeLineDistances();
  l.renderOrder = 2001;
  l.frustumCulled = false;
  return l;
}
new ResizeObserver(resize).observe(container);
resize();

// WASD + E/Q fly movement; the orbit target moves with the camera so orbiting keeps working
const held = new Set();
const FLY_KEYS = new Set(["KeyW", "KeyA", "KeyS", "KeyD", "KeyE", "KeyQ"]);
const typing = (e) => e.target.closest?.("input, textarea, select") || !document.getElementById("modal-pick").classList.contains("hidden") || !document.getElementById("modal-ask").classList.contains("hidden");
window.addEventListener("keydown", (e) => {
  if (!FLY_KEYS.has(e.code) || typing(e) || e.ctrlKey || e.metaKey || e.altKey) return;
  held.add(e.code);
  e.preventDefault();
});
window.addEventListener("keyup", (e) => held.delete(e.code));
window.addEventListener("blur", () => held.clear());
let flySpeed = 1.5; // m/s, rescaled to the scene on every fit
let lastFrame = performance.now();
const fwd = new THREE.Vector3(), right = new THREE.Vector3(), move = new THREE.Vector3();
function fly(dt) {
  if (!held.size) return;
  camera.getWorldDirection(fwd);
  fwd.y = 0;
  // looking straight down: use the screen's up direction as "forward"
  if (fwd.lengthSq() < 1e-4) fwd.set(0, 1, 0).applyQuaternion(camera.quaternion).setY(0);
  fwd.normalize();
  right.crossVectors(fwd, new THREE.Vector3(0, 1, 0)).normalize();
  move.set(0, 0, 0);
  if (held.has("KeyW")) move.add(fwd);
  if (held.has("KeyS")) move.sub(fwd);
  if (held.has("KeyD")) move.add(right);
  if (held.has("KeyA")) move.sub(right);
  if (!walk) { // landed: no flying up or down
    if (held.has("KeyE")) move.y += 1;
    if (held.has("KeyQ")) move.y -= 1;
  }
  if (move.lengthSq()) move.normalize().multiplyScalar((walk ? 1.4 : flySpeed) * dt * (shiftHeld ? 3 : 1));
  if (walk) {
    if (move.lengthSq()) followFloor(camera.position.x + move.x, camera.position.z + move.z);
    move.y = walk.floorY + walk.eye - camera.position.y;
  }
  if (!move.lengthSq()) return;
  camera.position.add(move);
  controls.target.add(move);
}

// ---- floor landing: stand on a clicked floor point at eye height and walk (WASD), drag to look around
let walk = null;
const downRay = new THREE.Raycaster();
// floor under (x, z): the highest model surface below the knees, or the scan floor / last floor
function floorBelow(x, z, fromY) {
  downRay.set(new THREE.Vector3(x, fromY, z), new THREE.Vector3(0, -1, 0));
  const meshes = state.refs.filter((r) => r.type === "bim" && r.group.visible).flatMap((r) => r.group.children.filter((o) => o.isMesh));
  const hit = downRay.intersectObjects(meshes, false)[0];
  return hit ? hit.point.y : null;
}
// keep the feet on the model's floors while walking: steps up to 0.45 m and down to 1.2 m (stairs, slabs)
let lastFollow = 0;
// the floor you see: the visible scan's floor under (x, z), else the model's
function floorAt(x, z, fromY) {
  const s = state.scans.find((o) => o === state.activeScan && o.group.visible) || state.scans.find((o) => o.group.visible);
  const y = s ? scanFloorAt(s, x, z, fromY) : null;
  return y ?? floorBelow(x, z, fromY);
}
function followFloor(x, z) {
  const now = performance.now();
  if (now - lastFollow < 80) return;
  lastFollow = now;
  const y = floorAt(x, z, walk.floorY + 0.45);
  if (y != null && y > walk.floorY - 1.2) walk.floorY = y;
}
// scan floor under (x, z): a low percentile of the splats within 0.6 m around the feet, so floaters and
// clutter above the floor do not lift the eye; the detected floor plane wins when it agrees
function scanFloorAt(s, x, z, belowY) {
  s._floorSample ??= sampleSplats(s.mesh, s.count, 300_000, false);
  s.group.updateMatrixWorld(true);
  const P = s._floorSample, e = s.mesh.matrixWorld.elements, ys = [];
  for (let i = 0; i < P.length; i += 3) {
    const px = P[i], py = P[i + 1], pz = P[i + 2];
    const wx = e[0] * px + e[4] * py + e[8] * pz + e[12], wz = e[2] * px + e[6] * py + e[10] * pz + e[14];
    if ((wx - x) ** 2 + (wz - z) ** 2 > 0.36) continue;
    const wy = e[1] * px + e[5] * py + e[9] * pz + e[13];
    if (wy < belowY) ys.push(wy);
  }
  const plane = s.floorLocal ? scanFloorY(s) : null;
  if (ys.length < 20) return plane;
  ys.sort((a, b) => a - b);
  const y = ys[Math.floor(ys.length * 0.1)];
  return plane != null && Math.abs(plane - y) < 0.3 ? plane : y;
}
function landAt(p, floorY) {
  if (!walk) walk = { saved: { pos: camera.position.clone(), target: controls.target.clone(), fov: camera.fov }, eye: 1.5 };
  walk.floorY = floorY;
  const dir = new THREE.Vector3();
  camera.getWorldDirection(dir);
  dir.y = 0;
  if (dir.lengthSq() < 1e-6) dir.set(0, 0, -1);
  dir.normalize();
  camera.position.set(p.x, floorY + walk.eye, p.z);
  controls.target.copy(camera.position).addScaledVector(dir, 0.05); // orbit about a point just ahead = look around
  camera.fov = 60;
  camera.near = 0.02;
  camera.updateProjectionMatrix();
  controls.enableZoom = false;
  controls.enablePan = false;
  controls.rotateSpeed = 0.35; // drag grabs the scene, like a street-view panorama
  document.getElementById("btn-land").classList.add("bg-zzikbang", "text-white");
}
function exitWalk(restore = true) {
  if (!walk) return;
  const w = walk;
  walk = null;
  controls.enableZoom = true;
  controls.enablePan = true;
  controls.rotateSpeed = 1;
  camera.fov = w.saved.fov;
  camera.updateProjectionMatrix();
  if (restore) { camera.position.copy(w.saved.pos); controls.target.copy(w.saved.target); }
  document.getElementById("btn-land").classList.remove("bg-zzikbang", "text-white");
}
let shiftHeld = false;
window.addEventListener("keydown", (e) => { if (e.key === "Shift") shiftHeld = true; });
window.addEventListener("keyup", (e) => { if (e.key === "Shift") shiftHeld = false; });

renderer.setAnimationLoop(() => {
  const now = performance.now();
  fly(Math.min(0.25, (now - lastFrame) / 1000)); // keep speed when heavy scans drop the frame rate
  lastFrame = now;
  controls.update();
  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
});

// =====================================================================
// 2. State & small UI helpers
// =====================================================================
const state = {
  scans: [],      // {id, name, date, group, mesh, visible, opacity, count}
  refs: [],       // drawings & BIM models, see addDrawing / addModel
  activeScan: null,
  activeRef: null,
  layer: "all",
  scanOpacity: 1,
  refOpacity: 0.8,
};

let toastTimer;
function toast(msg, ms = 3200) {
  const t = $("toast");
  t.textContent = msg;
  t.classList.remove("hidden");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.add("hidden"), ms);
}
function loader(on, msg) {
  $("canvas-loader").classList.toggle("hidden", !on);
  if (msg) $("loader-text").textContent = msg;
}
const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c]);
const today = () => new Date().toISOString().slice(0, 10);

// in-page dialog (window.prompt/confirm are blocked in some embeds)
function ask(title, body, fields = [], { info = false } = {}) {
  return new Promise((resolve) => {
    const m = $("modal-ask");
    $("ask-cancel").classList.toggle("hidden", info);
    $("ask-title").textContent = title;
    $("ask-body").textContent = body || "";
    $("ask-fields").innerHTML = fields.map((f, i) =>
      `<label class="block"><span class="text-[11px] font-semibold text-gray-500">${esc(f.label)}</span>
        <input data-i="${i}" type="${f.type || "number"}" value="${esc(f.value ?? "")}" class="inp mt-0.5" /></label>`).join("");
    m.classList.remove("hidden");
    m.classList.add("flex");
    const inputs = [...m.querySelectorAll("input")];
    inputs[0]?.focus();
    inputs[0]?.select?.();
    const done = (ok) => {
      m.classList.add("hidden");
      m.classList.remove("flex");
      window.removeEventListener("keydown", onKey, true);
      resolve(ok ? inputs.map((i) => i.value) : null);
    };
    const onKey = (e) => {
      if (e.key === "Enter") { e.preventDefault(); done(true); }
      if (e.key === "Escape") { e.stopPropagation(); done(false); }
    };
    window.addEventListener("keydown", onKey, true);
    $("ask-ok").onclick = () => done(true);
    $("ask-cancel").onclick = () => done(false);
  });
}

// ---- alignment lock: a locked scan and every reference aligned to it cannot be moved
function lockOwner(obj) {
  return state.scans.find((s) => s.locked && (s.group === obj || state.refs.some((o) => o.group === obj && s.linked?.has(o.id))));
}
let lockToastAt = 0;
function lockedToast() {
  if (performance.now() - lockToastAt < 1500) return;
  lockToastAt = performance.now();
  toast("정렬이 고정되어 있습니다 — 상단의 자물쇠를 눌러 고정을 풀어 주세요", 3500);
}
// true (and a notice) when this action would move something locked
function blockedByLock(s, r) {
  if (!s?.locked || (r && !s.linked?.has(r.id))) return false;
  lockedToast();
  return true;
}
// world-space similarity applied to an object: world = M * old
function applyWorld(obj, M) {
  if (lockOwner(obj)) return lockedToast();
  obj.updateMatrix();
  obj.matrix.clone().premultiply(M).decompose(obj.position, obj.quaternion, obj.scale);
  obj.updateMatrixWorld(true);
}
function aboutPoint(p, M) {
  return new THREE.Matrix4().makeTranslation(p.x, p.y, p.z).multiply(M).multiply(new THREE.Matrix4().makeTranslation(-p.x, -p.y, -p.z));
}
const yawOf = (q) => new THREE.Euler().setFromQuaternion(q, "YXZ").y;

function maxTex() { return Math.min(8192, renderer.capabilities.maxTextureSize); }

function refreshEmpty() {
  $("empty-state").classList.toggle("hidden", !!(state.scans.length || state.refs.length));
}
function applyVisibility() {
  const L = state.layer || "all";
  for (const s of state.scans) s.group.visible = s.visible && (L === "all" || L === "scan");
  for (const r of state.refs) r.group.visible = r.visible && (L === "all" || L === (r.type === "bim" ? "bim" : "dwg"));
}

// =====================================================================
// 3. Scans (viewing only — needed as the alignment target)
// =====================================================================
// framing, drawing placement and section boxes use the scan's core, not its far background shell
function scanBox(s) {
  s.group.updateMatrixWorld(true);
  return (s.core?.box ?? s.mesh.getBoundingBox(true)).clone().applyMatrix4(s.mesh.matrixWorld);
}

// hide splats outside the core: an inverted box SDF that zeroes opacity, attached to the mesh
function setCrop(s, on) {
  if (!s.core) return;
  if (on && !s.cropEdit) {
    const edit = new SplatEdit({ rgbaBlendMode: SplatEditRgbaBlendMode.MULTIPLY, softEdge: 0.01, invert: true });
    const sdf = new SplatEditSdf({ type: SplatEditSdfType.BOX, opacity: 0, color: new THREE.Color(1, 1, 1) });
    s.core.box.getCenter(sdf.position);
    s.core.box.getSize(sdf.scale).multiplyScalar(0.5); // box SDF sizes are half extents
    edit.add(sdf);
    s.mesh.add(edit);
    s.cropEdit = edit;
  }
  if (s.cropEdit) s.cropEdit.visible = on;
  s.crop = on;
}
function inCore(s, p) {
  if (!s.crop || !s.core) return true;
  s.mesh.updateMatrixWorld(true);
  return s.core.box.containsPoint(p.clone().applyMatrix4(s.mesh.matrixWorld.clone().invert()));
}
const scanCenter = (s) => scanBox(s).getCenter(new THREE.Vector3());

const nextFrame = () => new Promise((r) => requestAnimationFrame(() => setTimeout(r, 0)));

// plain point clouds become small round splats sized from the point spacing (flat discs when normals exist)
function pointCloudMesh(pc) {
  const spacing = estimateSpacing(pc.positions, pc.count);
  const sigma = spacing * 0.6;
  const mesh = new SplatMesh({
    maxSplats: pc.count,
    constructSplats: (splats) => {
      const c = new THREE.Vector3(), round = new THREE.Vector3(sigma, sigma, sigma), disc = new THREE.Vector3(sigma, sigma, sigma * 0.15);
      const q = new THREE.Quaternion(), ident = new THREE.Quaternion(), zAxis = new THREE.Vector3(0, 0, 1), nv = new THREE.Vector3(), col = new THREE.Color();
      const { positions: P, colors: C, intensity: I, normals: N } = pc;
      for (let i = 0; i < pc.count; i++) {
        c.fromArray(P, i * 3);
        if (C) col.setRGB(C[i * 3], C[i * 3 + 1], C[i * 3 + 2]);
        else if (I) col.setRGB(I[i], I[i], I[i]);
        else col.setRGB(0.78, 0.78, 0.78);
        let scale = round, rot = ident;
        if (N) {
          nv.fromArray(N, i * 3);
          if (nv.lengthSq() > 0.25) { q.setFromUnitVectors(zAxis, nv.normalize()); scale = disc; rot = q; }
        }
        splats.pushSplat(c, scale, rot, 1, col);
      }
    },
  });
  return { mesh, spacing };
}

async function loadScanFile(f) {
  const t0 = performance.now();
  const bytes = new Uint8Array(await f.arrayBuffer());
  const ext = f.name.split(".").pop().toLowerCase();
  let mesh, kind = "gaussian", info = "", spacing = null, offset = null, header = null;
  if (ext === "ply") {
    const h = (header = parsePlyHeader(bytes));
    if (!h) throw new Error("PLY 헤더를 읽을 수 없습니다");
    if (!isGaussianPly(h)) {
      loader(true, `포인트클라우드 PLY (${f.name}) 읽는 중...`);
      await nextFrame();
      const pc = readPointCloud(bytes, h);
      if (!pc.count) throw new Error("PLY에 읽을 수 있는 점이 없습니다");
      ({ mesh, spacing } = pointCloudMesh(pc));
      kind = "pointcloud";
      offset = pc.offset.some((v) => v !== 0) ? pc.offset : null;
      info = `포인트클라우드 · ${h.format === "ascii" ? "ASCII" : "Binary"}` + (pc.total > pc.count ? ` · ${pc.total.toLocaleString()}점 중 ${pc.count.toLocaleString()}점 표시` : "") +
        (pc.colors ? "" : pc.intensity ? " · 강도(intensity) 색상" : " · 색상 없음");
    } else info = h.elements.some((e) => e.name === "chunk") ? "3DGS · 압축 PLY" : "3DGS PLY";
  } else info = `3DGS ${ext.toUpperCase()}`;
  if (!mesh) mesh = new SplatMesh({ fileBytes: bytes, fileName: f.name });
  await mesh.initialized;
  const group = new THREE.Group();
  group.add(mesh);
  scene.add(group);
  const m = f.name.match(/(20\d\d)[-_.]?(\d\d)[-_.]?(\d\d)/);
  const s = {
    id: uid++, name: f.name, group, mesh, kind, info, spacing, offset, floor: null, visible: true, opacity: state.scanOpacity,
    fileSize: f.size, header, loadMs: Math.round(performance.now() - t0),
    date: m ? `${m[1]}-${m[2]}-${m[3]}` : new Date(f.lastModified).toISOString().slice(0, 10),
    count: mesh.packedSplats?.numSplats ?? mesh.extSplats?.numSplats ?? mesh.splats?.getNumSplats?.() ?? 0,
  };
  mesh.opacity = s.opacity;
  s.core = computeCore(sampleSplats(mesh, s.count, 200_000, false));
  if (s.core.frac < 0.985) setCrop(s, $("bg-crop").checked);
  state.scans.push(s);
  if (scanColor.mode !== "orig") setTimeout(() => applyScanColors(), 0);
  state.activeScan = s;
  applyVisibility();
  return s;
}

async function onScanFiles(files, restore = null) {
  for (const f of files) {
    loader(true, `3DGS 파일 (${f.name}) 로딩 중...`);
    let s = null;
    try {
      s = await loadScanFile(f);
      keepLayer(s, "scan", f, restore);
      if (s.offset) toast(`큰 좌표값을 원점 기준으로 옮겼습니다 (원점 오프셋 ${s.offset.map((v) => v.toFixed(2)).join(", ")})`, 6000);
    } catch (e) {
      console.error(e);
      toast(`스캔을 열지 못했습니다: ${f.name} (${e.message || e})`, 6000);
    } finally {
      loader(false);
    }
    if (!s) continue;
    if (restore) restoreScan(s, restore);
    else if ($("floor-auto").checked) await runFloor(s);
    else fitView();
  }
  renderAll();
}

// ---- diagnostics: everything needed to debug a scan without having the file itself
function scanDiagnostics(s) {
  const r3 = (v) => v.toArray().map((x) => +x.toFixed(3));
  const local = s.mesh.getBoundingBox(true); // every splat, background shell included
  const world = scanBox(s);
  const h = s.header;
  return {
    app: "zzikbang-scan-overlay", file: s.name, sizeMB: +(s.fileSize / 1048576).toFixed(1), loadMs: s.loadMs,
    kind: s.kind, info: s.info, count: s.count, spacing: s.spacing, recentredOffset: s.offset,
    ply: h && { format: h.format, elements: h.elements.map((e) => ({ name: e.name, count: e.count, props: e.props.map((p) => (p.list ? `list ${p.type} ${p.name}` : `${p.type} ${p.name}`)).join(", ") })) },
    bboxFile: { min: r3(local.min), max: r3(local.max), size: r3(local.getSize(local.min.clone())) },
    bboxCoreWorld: { min: r3(world.min), max: r3(world.max), size: r3(world.getSize(world.min.clone())) },
    core: s.core && { fracInside: +s.core.frac.toFixed(3), min: r3(s.core.box.min), max: r3(s.core.box.max), cropOn: !!s.crop },
    floor: s.floor && {
      chosen: s.floor.idx, info: s.floor.info,
      candidates: s.floor.cands.slice(0, 8).map((c) => ({
        method: c.method, up: r3(c.up), frac: +c.frac.toFixed(3), aboveFrac: +c.aboveFrac.toFixed(3),
        extent: +c.extent.toFixed(3), quality: c.quality != null ? +c.quality.toFixed(2) : undefined, score: +c.score.toFixed(4),
      })),
    },
    floorMode: $("floor-mode").value, metric: !!s.metric,
    gpu: (() => { try { const gl = renderer.getContext(); const ext = gl.getExtension("WEBGL_debug_renderer_info"); return ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : "?"; } catch { return "?"; } })(),
    ua: navigator.userAgent,
  };
}
async function showDiagnostics(s) {
  const text = JSON.stringify(scanDiagnostics(s), null, 1);
  let copied = false;
  try { await navigator.clipboard.writeText(text); copied = true; } catch { /* clipboard may be blocked in embeds */ }
  const m = $("modal-ask");
  $("ask-title").textContent = "스캔 진단 정보";
  $("ask-body").textContent = copied ? "클립보드에 복사했습니다. 채팅에 그대로 붙여 넣어 주세요." : "아래 내용을 전체 선택해 복사한 뒤 채팅에 붙여 넣어 주세요.";
  $("ask-fields").innerHTML = `<textarea readonly class="inp font-mono text-[10px]" style="height: 16rem">${esc(text)}</textarea>`;
  $("ask-cancel").classList.add("hidden");
  m.classList.remove("hidden");
  m.classList.add("flex");
  m.querySelector("textarea").select();
  $("ask-ok").onclick = () => { m.classList.add("hidden"); m.classList.remove("flex"); };
}

// ---- floor recognition
async function runFloor(s, mode = $("floor-mode").value) {
  loader(true, "바닥 인식 중... (평면 검출)");
  await nextFrame();
  try {
    // always detect from the pre-floor pose so re-running and cycling candidates is stable
    if (s.floor?.base) restoreMatrix(s.group, s.floor.base);
    s.group.updateMatrix();
    const base = s.group.matrix.clone();
    s.group.updateMatrixWorld(true);
    let P = sampleSplats(s.mesh, s.count);
    if (s.core) P = filterToBox(P, s.core.box, s.mesh.matrixWorld);
    const cands = detectFloor(P, mode, s.kind === "pointcloud");
    if (!cands.length) {
      s.floor = null;
  s.floorLocal = null; // manual corrections invalidate the remembered floor plane
      toast("바닥 평면을 찾지 못했습니다 — '바닥 3점 수평'으로 직접 지정하세요", 5000);
      return;
    }
    s.floor = { base, cands, idx: 0 };
    s.metric = mode === "scaniverse" || (mode === "gravity" && s.kind === "pointcloud") || s.metric;
    applyFloorCandidate(s, 0);
  } catch (e) {
    console.error(e);
    toast(`바닥 인식 실패: ${e.message || e}`, 5000);
  } finally {
    loader(false);
  }
}
function restoreMatrix(obj, m) {
  if (lockOwner(obj)) return lockedToast();
  m.decompose(obj.position, obj.quaternion, obj.scale);
  obj.updateMatrixWorld(true);
}
function applyFloorCandidate(s, idx) {
  const f = s.floor;
  f.idx = idx;
  restoreMatrix(s.group, f.base);
  const c = f.cands[idx];
  applyWorld(s.group, floorTransform(c));
  const fc = scanBox(s).getCenter(new THREE.Vector3()).setY(0);
  s.floorLocal = fc.applyMatrix4(s.mesh.matrixWorld.clone().invert());
  const tilt = c.up.angleTo(UP) / DEG;
  f.info = `${c.method === "plane" ? "평면 검출" : "중력 정렬"} · 후보 ${idx + 1}/${f.cands.length} · 바닥 점 ${(c.frac * 100).toFixed(0)}% · 보정 ${tilt.toFixed(1)}°`;
  flashFloor(s);
  fitView();
  renderAll();
  toast(`바닥 인식: ${f.info} — 바닥을 높이 0 m에 맞췄습니다`, 4500);
}
// orange sheet on the detected floor (y = 0) that fades out
function flashFloor(s) {
  const b = scanBox(s);
  const w = Math.max(0.5, b.max.x - b.min.x), d = Math.max(0.5, b.max.z - b.min.z);
  const mat = new THREE.MeshBasicMaterial({ color: 0xff6b00, transparent: true, opacity: 0.35, depthWrite: false, side: THREE.DoubleSide });
  const sheet = new THREE.Mesh(new THREE.PlaneGeometry(w, d), mat);
  sheet.rotation.x = -Math.PI / 2;
  sheet.position.set((b.min.x + b.max.x) / 2, 0.002, (b.min.z + b.max.z) / 2);
  const edge = new THREE.LineSegments(new THREE.EdgesGeometry(sheet.geometry), new THREE.LineBasicMaterial({ color: 0xff6b00, transparent: true }));
  sheet.add(edge);
  helpers.add(sheet);
  const t0 = performance.now();
  sheet.onBeforeRender = () => {
    const k = 1 - (performance.now() - t0) / 3500;
    mat.opacity = 0.35 * Math.max(0, k);
    edge.material.opacity = Math.max(0, k);
    if (k <= 0) setTimeout(() => helpers.remove(sheet));
  };
}

function removeScan(s) {
  if (s.persistId) persistDelete(s.persistId);
  scene.remove(s.group);
  dropLabels(s.group); // marker labels are DOM nodes and would otherwise stay on screen
  s.mesh.dispose?.();
  state.scans = state.scans.filter((o) => o !== s);
  if (state.activeScan === s) state.activeScan = state.scans.at(-1) || null;
  renderAll();
}

function renderScans() {
  // once something is loaded the big drop zone gives way to a compact add button
  $("dropzone-ply").classList.toggle("hidden", state.scans.length > 0);
  $("btn-add-ply").classList.toggle("hidden", !state.scans.length);
  const list = $("scan-list");
  list.innerHTML = "";
  for (const s of [...state.scans].sort((a, b) => a.date.localeCompare(b.date))) {
    const el = document.createElement("div");
    el.className = `rounded-xl border p-2.5 space-y-2 ${s === state.activeScan ? "border-zzikbang bg-orange-50/40" : "border-gray-200 bg-white"}`;
    el.innerHTML = `
      <div class="flex items-center gap-2">
        <button data-a="vis" class="text-xs w-5 ${s.visible ? "text-zzikbang" : "text-gray-300"}" title="표시"><i class="fa-solid ${s.visible ? "fa-eye" : "fa-eye-slash"}"></i></button>
        <button data-a="sel" class="flex-1 min-w-0 text-left">
          <p class="text-xs font-bold text-gray-800 truncate"><span class="text-[9px] font-extrabold px-1 rounded mr-1 ${s.kind === "pointcloud" ? "bg-sky-50 text-sky-600" : "bg-orange-50 text-zzikbang"}">${s.kind === "pointcloud" ? "PC" : "3DGS"}</span>${esc(s.name)}</p>
          <p class="text-[10px] text-gray-400 truncate">${s.count ? s.count.toLocaleString() + (s.kind === "pointcloud" ? " pts · " : " splats · ") : ""}${s.date} · ${esc(s.info || "")}</p>
        </button>
        ${s.core && s.core.frac < 0.985 ? `<button data-a="crop" class="text-xs ${s.crop ? "text-zzikbang" : "text-gray-300"}" title="${s.crop ? "원거리 배경 숨김 중 (눌러서 표시)" : "원거리 배경 표시 중 (눌러서 숨김)"}"><i class="fa-solid fa-crop-simple"></i></button>` : ""}
        ${s.markers?.length ? `<button data-a="mk" class="text-xs ${s.showMarkers !== false ? "text-red-600" : "text-gray-300"}" title="마커 ${s.markers.length}개 ${s.showMarkers !== false ? "숨기기" : "보이기"}"><i class="fa-solid fa-location-dot"></i></button>` : ""}
        <button data-a="diag" class="text-gray-300 hover:text-zzikbang text-xs" title="진단 정보 복사"><i class="fa-solid fa-circle-info"></i></button>
        <button data-a="floor" class="text-[9px] font-bold px-1.5 py-0.5 rounded whitespace-nowrap ${s.floor ? "bg-emerald-50 text-emerald-600" : "bg-gray-100 text-gray-500 hover:text-zzikbang"}" title="${s.floor ? esc(s.floor.info) : "바닥 자동 인식"}">${s.floor ? "바닥 ✓" : "바닥 인식"}</button>
        <button data-a="del" class="text-gray-300 hover:text-red-500 text-xs" title="제거"><i class="fa-solid fa-xmark"></i></button>
      </div>
      <div class="flex items-center gap-2">
        <span class="text-[10px] text-gray-400 w-10">투명도</span>
        <input data-a="op" type="range" class="sm flex-1" min="0" max="100" value="${Math.round(s.opacity * 100)}" />
        <span class="text-[10px] font-mono font-bold text-zzikbang w-8 text-right">${Math.round(s.opacity * 100)}%</span>
      </div>`;
    el.querySelector('[data-a="vis"]').onclick = () => { s.visible = !s.visible; applyVisibility(); renderAll(); };
    el.querySelector('[data-a="sel"]').onclick = () => { state.activeScan = s; renderAll(); };
    el.querySelector('[data-a="floor"]').onclick = () => { state.activeScan = s; runFloor(s); };
    el.querySelector('[data-a="diag"]').onclick = () => showDiagnostics(s);
    el.querySelector('[data-a="crop"]')?.addEventListener("click", () => { setCrop(s, !s.crop); renderAll(); });
    el.querySelector('[data-a="mk"]')?.addEventListener("click", () => {
      s.showMarkers = s.showMarkers === false;
      if (s.markerPins) s.markerPins.visible = s.showMarkers;
      renderAll();
    });
    el.querySelector('[data-a="del"]').onclick = () => removeScan(s);
    const op = el.querySelector('[data-a="op"]');
    op.oninput = () => { s.opacity = op.value / 100; s.mesh.opacity = s.opacity; op.nextElementSibling.textContent = op.value + "%"; };
    list.appendChild(el);
  }
}

// =====================================================================
// 4. Reference layers: 2D drawings (image / PDF / DWG / DXF) and BIM (IFC)
// =====================================================================
const drawVert = `varying vec2 vUv; void main(){ vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`;
const drawFrag = `
  uniform sampler2D map; uniform float opacity; uniform bool lineOnly; uniform bool useTint; uniform vec3 tint;
  varying vec2 vUv;
  void main(){
    vec4 c = texture2D(map, vUv);
    float lum = dot(c.rgb, vec3(0.299, 0.587, 0.114));
    float a = lineOnly ? clamp((1.0 - lum) * 1.6, 0.0, 1.0) : 1.0;
    vec3 col = useTint ? mix(c.rgb, tint, lineOnly ? 1.0 : 0.6 * (1.0 - lum)) : c.rgb;
    gl_FragColor = vec4(col, a * opacity * c.a);
    if (gl_FragColor.a < 0.01) discard;
    #include <colorspace_fragment>
  }`;

function addDrawing({ name, kind, canvas: cv, ppm, dpi, calibrated, info, snap }) {
  const r = {
    id: uid++, type: "drawing", kind, name, info, canvas: cv, pxW: cv.width, pxH: cv.height, snap: snap ? snapIndex(snap) : null,
    ppm, dpi: dpi || null, calibrated: !!calibrated, orient: "plan", visible: true,
    opts: { opacity: state.refOpacity, lineOnly: true, useTint: true, tint: "#ff6b00", order: "top" }, // brand orange lines
    refPts: [], group: new THREE.Group(), pins: new THREE.Group(),
  };
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = renderer.capabilities.getMaxAnisotropy();
  r.mat = new THREE.ShaderMaterial({
    uniforms: { map: { value: tex }, opacity: { value: 1 }, lineOnly: { value: true }, useTint: { value: true }, tint: { value: new THREE.Color() } },
    vertexShader: drawVert, fragmentShader: drawFrag, transparent: true, depthWrite: false, side: THREE.DoubleSide,
  });
  r.mesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), r.mat);
  r.group.add(r.mesh, r.pins);
  // place a new drawing at the floor level of the current scan, centred on it
  if (state.activeScan) {
    const b = scanBox(state.activeScan);
    r.group.position.set((b.min.x + b.max.x) / 2, 0, (b.min.z + b.max.z) / 2);
  }
  scene.add(r.group);
  rebuildDrawing(r);
  addRef(r);
  return r;
}

function rebuildDrawing(r) {
  const W = r.pxW / r.ppm, H = r.pxH / r.ppm;
  r.mesh.geometry.dispose();
  r.mesh.geometry = new THREE.PlaneGeometry(W, H);
  r.mesh.rotation.set(r.orient === "plan" ? -Math.PI / 2 : 0, 0, 0);
  const o = r.opts, u = r.mat.uniforms;
  u.opacity.value = o.opacity;
  u.lineOnly.value = o.lineOnly;
  u.useTint.value = o.useTint;
  u.tint.value.set(o.tint);
  r.mat.depthTest = o.order !== "top";
  r.mesh.renderOrder = o.order === "top" ? 999 : -1;
  r.mat.needsUpdate = true;
  rebuildPins(r);
}

// drawing pixel -> drawing-group local metres (image top = -Z on plans, +Y on elevations)
function pixelToLocal(r, px, py) {
  const W = r.pxW / r.ppm, H = r.pxH / r.ppm, x = px / r.ppm - W / 2;
  return r.orient === "plan" ? new THREE.Vector3(x, 0, py / r.ppm - H / 2) : new THREE.Vector3(x, H / 2 - py / r.ppm, 0);
}
function refPointWorld(r, p) {
  r.group.updateMatrixWorld(true);
  const local = r.type === "drawing" ? pixelToLocal(r, p.px, p.py) : new THREE.Vector3().fromArray(p.local);
  return local.applyMatrix4(r.group.matrixWorld);
}

function addModel({ name, group, materials, objects, pieces, coordMatrix, ifcText, info, quantities, units, kind = "ifc" }) {
  const r = {
    id: uid++, type: "bim", kind, name, info, group, materials, objects: objects || [], pieces: pieces || [], visible: true, calibrated: true,
    coordMatrix, ifcText, quantities: quantities || new Map(), units,
    markerGroups: markerGroups(objects || []),
    opts: { opacity: state.refOpacity, ghost: true, color: "#d4d7dc" }, refPts: [], pins: new THREE.Group(),
  };
  group.add(r.pins);
  scene.add(group);
  r.levels = floorLevels(r);
  // default: the lowest level carrying a real floor area (skips footings and small ledges)
  const maxArea = Math.max(0, ...r.levels.map((l) => l.area));
  r.levelIdx = Math.max(0, r.levels.findIndex((l) => l.area >= maxArea * 0.25));
  styleModel(r, r.opts);
  applyFront(r);
  buildModelEdges(r);
  for (const m of materials) m.clippingPlanes = cutPlanes();
  addRef(r);
  return r;
}

let refFile = null, refRestore = null; // file being opened by onRefFile (addRef stores it)
function addRef(r) {
  if (refFile) keepLayer(r, "ref", refFile, refRestore);
  state.refs.push(r);
  state.activeRef = r;
  applyVisibility();
  renderAll();
  if (!state.scans.length) fitView();
}
function removeRef(r) {
  if (r.persistId) persistDelete(r.persistId);
  for (const s of state.scans) {
    s.linked?.delete(r.id);
    if (s.anchor === r.id) s.anchor = [...(s.linked || [])][0] ?? null;
  }
  scene.remove(r.group);
  dropLabels(r.group);
  state.refs = state.refs.filter((o) => o !== r);
  if (state.activeRef === r) state.activeRef = state.refs.at(-1) || null;
  renderAll();
}
function styleRef(r) {
  if (r.type === "drawing") rebuildDrawing(r);
  else { styleModel(r, r.opts); applyFront(r); styleEdges(r); }
}
// ---- outlines of every IFC element (per element, so they stay complete where elements touch)
function buildModelEdges(r) {
  const byId = new Map(); // element id -> triangle positions
  const loose = [];
  for (const m of r.group.children) {
    const g = m.isMesh && m.geometry;
    if (!g?.attributes?.position) continue;
    const ids = g.userData.ids;
    if (!ids) { loose.push(m); continue; }
    const pos = g.attributes.position.array, idx = g.index?.array, nTri = (idx ? idx.length : pos.length / 3) / 3;
    for (let t = 0; t < nTri; t++) {
      let arr = byId.get(ids[t]);
      if (!arr) byId.set(ids[t], (arr = []));
      for (let k = 0; k < 3; k++) { const v = idx ? idx[t * 3 + k] : t * 3 + k; arr.push(pos[v * 3], pos[v * 3 + 1], pos[v * 3 + 2]); }
    }
  }
  r.elemEdges ??= new Map();
  const all = [];
  const edgesOf = (tri) => {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(tri, 3));
    const e = new THREE.EdgesGeometry(g, 25).attributes.position.array;
    g.dispose();
    return e;
  };
  for (const [id, tri] of byId) {
    const e = r.elemEdges.get(id) ?? edgesOf(tri);
    r.elemEdges.set(id, e); // the ruler snaps to the same outlines
    for (let i = 0; i < e.length; i++) all.push(e[i]);
  }
  for (const m of loose) {
    m.userData.edges ??= new THREE.EdgesGeometry(m.geometry, 25).attributes.position.array;
    for (const v of m.userData.edges) all.push(v);
  }
  if (!all.length) return;
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(all, 3));
  r.edgeMat = new THREE.LineBasicMaterial({ color: 0x374151, transparent: true, depthWrite: false, clippingPlanes: cutPlanes() });
  r.edges = new THREE.LineSegments(geo, r.edgeMat);
  r.edges.renderOrder = 1;
  r.edges.userData.dimOverlay = true; // not a pickable surface
  r.group.add(r.edges);
  styleEdges(r);
}
function styleEdges(r) {
  if (!r.edgeMat) return;
  r.edgeMat.opacity = Math.min(1, r.opts.opacity + 0.25);
  r.edges.visible = r.opts.opacity > 0.02;
}

async function onRefFile(f, restore = null) {
  refFile = f;
  refRestore = restore;
  const n0 = state.refs.length;
  try {
    await openRefFile(f, restore);
  } finally {
    refFile = refRestore = null;
  }
  const r = state.refs.length > n0 ? state.refs.at(-1) : null;
  if (r && restore) restoreRef(r, restore);
}
async function openRefFile(f, restore) {
  const ext = f.name.split(".").pop().toLowerCase();
  if (ext === "rvt") {
    await ask("RVT 파일은 직접 열 수 없습니다",
      "Revit(.rvt)은 Autodesk 전용 포맷이라 브라우저에서 해석할 수 있는 공개 라이브러리가 없습니다.\n\n" +
      "Revit에서 아래 중 하나로 내보내 업로드해 주세요.\n" +
      "• 3D 모델 겹쳐보기: 파일 → 내보내기 → IFC (.ifc)\n" +
      "• 평면/단면 도면: 파일 → 내보내기 → CAD 형식 → DWG, 또는 PDF", [], { info: true });
    return;
  }
  if (ext === "skp") {
    await ask("SKP 파일은 직접 열 수 없습니다",
      "SketchUp(.skp)은 전용 포맷이라 브라우저에서 해석할 수 있는 공개 라이브러리가 없습니다.\n\n" +
      "SketchUp에서 STL로 내보내 업로드해 주세요 (무료 웹 버전 포함).\n" +
      "• SketchUp 웹(무료): 메뉴 → 다운로드 → STL\n" +
      "• SketchUp Pro/데스크톱: 파일 → 내보내기 → 3D 모델 → STL (또는 DAE · OBJ · IFC)\n\n" +
      "STL은 단위 정보가 없어서, 올릴 때 단위(mm·m 등)를 확인하는 창이 뜹니다.", [], { info: true });
    return;
  }
  loader(true, `${ext.toUpperCase()} 도면 파일 파싱 중...`);
  try {
    if (MESH_EXT.includes(ext)) {
      const m = await loadMeshModel(f, {
        askUnit: async (guessMm, sizeText) => {
          if (restore?.unitMm) return restore.unitMm;
          loader(false);
          const v = await ask("모델 단위 확인", `이 형식은 단위 정보가 없습니다.\n모델 크기: ${sizeText}\n\n크기로 추정한 값을 채워 두었습니다 (1000 = m, 1 = mm, 25.4 = inch).`, [{ label: "1 모델 단위 = ? mm", value: guessMm }]);
          loader(true, "3D 모델 변환 중...");
          lastUnitMm = v ? parseFloat(v[0]) : null;
          return lastUnitMm;
        },
      });
      if (!m) return;
      addModel({ name: f.name, ...m });
      toast(`${m.info} — 3D 모델 레이어로 올렸습니다`);
    } else if (ext === "ifc") {
      const m = await loadIfc(f, (p) => loader(true, `IFC 형상 생성 중... ${Math.round(p * 100)}%`));
      addModel({ name: f.name, ...m, ifcText: await f.text() });
      toast(`${m.info} — BIM 모델은 실척(m)으로 배치됩니다`);
    } else if (ext === "dwg" || ext === "dxf") {
      const res = await loadCad(f, {
        maxTex: maxTex(),
        askUnit: async (msg) => {
          if (restore?.unitMm) return restore.unitMm;
          loader(false);
          const v = await ask("도면 단위", msg, [{ label: "1 도면단위 = ? mm", value: 1 }]);
          lastUnitMm = v ? parseFloat(v[0]) : null;
          return lastUnitMm;
        },
      });
      if (!res) return;
      addDrawing({ name: f.name, kind: ext, canvas: res.canvas, ppm: res.ppm, calibrated: true, info: res.info, snap: res.snap });
      toast(res.info);
    } else if (ext === "pdf") {
      await loadPdf(f);
    } else {
      await loadImage(f);
    }
  } catch (e) {
    console.error(e);
    toast(`도면을 열지 못했습니다: ${e.message || e}`, 7000);
  } finally {
    loader(false);
  }
}

async function loadImage(f) {
  const img = await createImageBitmap(f);
  const k = Math.min(1, maxTex() / Math.max(img.width, img.height));
  const cv = document.createElement("canvas");
  cv.width = Math.round(img.width * k);
  cv.height = Math.round(img.height * k);
  const ctx = cv.getContext("2d");
  ctx.fillStyle = "#fff";
  ctx.fillRect(0, 0, cv.width, cv.height);
  ctx.drawImage(img, 0, 0, cv.width, cv.height);
  const dpi = 150 * k;
  addDrawing({ name: f.name, kind: "image", canvas: cv, dpi, ppm: dpi / 0.0254 / 100, info: `${img.width}×${img.height}px 이미지` });
  toast("이미지 도면은 축척 정보가 없습니다 — 왼쪽 도면 아래 축척에서 맞춰주세요");
}

let pdfjs;
async function loadPdf(f) {
  if (!pdfjs) {
    // window.__LIBS lets a self-hosted build point at local copies of the libraries
    pdfjs = await import(window.__LIBS?.pdf ?? "https://cdn.jsdelivr.net/npm/pdfjs-dist@4.10.38/build/pdf.min.mjs");
    pdfjs.GlobalWorkerOptions.workerSrc = window.__LIBS?.pdfWorker ?? "https://cdn.jsdelivr.net/npm/pdfjs-dist@4.10.38/build/pdf.worker.min.mjs";
  }
  const doc = await pdfjs.getDocument({ data: new Uint8Array(await f.arrayBuffer()) }).promise;
  let pageNo = 1;
  if (doc.numPages > 1) {
    loader(false);
    const v = await ask("PDF 페이지 선택", `이 PDF는 ${doc.numPages}페이지입니다.`, [{ label: "불러올 페이지", value: 1 }]);
    if (!v) return;
    pageNo = Math.min(doc.numPages, Math.max(1, parseInt(v[0], 10) || 1));
    loader(true, "PDF 렌더링 중...");
  }
  const page = await doc.getPage(pageNo);
  const vp1 = page.getViewport({ scale: 1 });
  const scale = Math.min(maxTex() / Math.max(vp1.width, vp1.height), 400 / 72);
  const vp = page.getViewport({ scale });
  const cv = document.createElement("canvas");
  cv.width = Math.floor(vp.width);
  cv.height = Math.floor(vp.height);
  const ctx = cv.getContext("2d");
  ctx.fillStyle = "#fff";
  ctx.fillRect(0, 0, cv.width, cv.height);
  await page.render({ canvasContext: ctx, viewport: vp }).promise;
  const dpi = 72 * scale; // PDF units are 1/72 inch of paper, so 1:N gives an exact scale
  const paper = `${Math.round(vp1.width / 72 * 25.4)}×${Math.round(vp1.height / 72 * 25.4)} mm 용지`;
  addDrawing({ name: f.name + (doc.numPages > 1 ? ` (p.${pageNo})` : ""), kind: "pdf", canvas: cv, dpi, ppm: dpi / 0.0254 / 100, info: paper });
  toast(`PDF ${paper} — 왼쪽 도면 아래 축척(1:N·DPI)을 입력하거나 자 버튼으로 2점 보정하세요`, 5000);
}

const MESH_BADGE = ["3D", "bg-teal-50 text-teal-600"];
const KIND_BADGE = { stl: ["STL", "bg-teal-50 text-teal-600"], dae: MESH_BADGE, kmz: MESH_BADGE, obj: MESH_BADGE, glb: MESH_BADGE, gltf: MESH_BADGE, fbx: MESH_BADGE, pdf: ["PDF", "bg-red-50 text-red-500"], dwg: ["DWG", "bg-blue-50 text-blue-600"], dxf: ["DXF", "bg-blue-50 text-blue-600"], ifc: ["IFC", "bg-emerald-50 text-emerald-600"], image: ["IMG", "bg-purple-50 text-purple-500"] };

function renderRefs() {
  const nDwg = state.refs.filter((o) => o.type === "drawing").length, nBim = state.refs.length - nDwg;
  $("dropzone-cad").classList.toggle("hidden", nDwg > 0);
  $("btn-add-cad").classList.toggle("hidden", !nDwg);
  $("dropzone-bim").classList.toggle("hidden", nBim > 0);
  $("btn-add-bim").classList.toggle("hidden", !nBim);
  $("dwg-list").innerHTML = "";
  $("bim-list").innerHTML = "";
  for (const r of state.refs) {
    const list = $(r.type === "bim" ? "bim-list" : "dwg-list");
    const [kb, kc] = KIND_BADGE[r.kind] || ["REF", "bg-gray-100 text-gray-500"];
    const el = document.createElement("div");
    el.className = `rounded-xl border p-2.5 space-y-2 ${r === state.activeRef ? "border-blue-400 bg-blue-50/30" : "border-gray-200 bg-white"}`;
    const calib = r.type === "bim" ? "" : r.calibrated
      ? `<span class="text-[9px] font-bold px-1 rounded bg-emerald-50 text-emerald-600">축척 OK</span>`
      : `<span class="text-[9px] font-bold px-1 rounded bg-amber-50 text-amber-600">축척 미보정</span>`;
    el.innerHTML = `
      <div class="flex items-center gap-2">
        <button data-a="vis" class="text-xs w-5 ${r.visible ? "text-blue-500" : "text-gray-300"}" title="표시"><i class="fa-solid ${r.visible ? "fa-eye" : "fa-eye-slash"}"></i></button>
        <button data-a="sel" class="flex-1 min-w-0 text-left">
          <p class="text-xs font-bold text-gray-800 truncate"><span class="text-[9px] font-extrabold px-1 rounded ${kc} mr-1">${kb}</span>${esc(r.name)}</p>
          <p class="text-[10px] text-gray-400 truncate">${calib} ${esc(r.info || "")}</p>
        </button>
        <input data-a="col" type="color" value="${r.type === "drawing" ? r.opts.tint : r.opts.color}" class="w-5 h-5 border-0 bg-transparent p-0 cursor-pointer" title="${r.type === "drawing" ? "선 색상" : "모델 색상 (단색)"}" />
        <button data-a="opt" class="text-gray-400 hover:text-gray-700 text-xs" title="표시 설정"><i class="fa-solid fa-gear"></i></button>
        <button data-a="del" class="text-gray-300 hover:text-red-500 text-xs" title="제거"><i class="fa-solid fa-xmark"></i></button>
      </div>
      <div class="flex items-center gap-2">
        <span class="text-[10px] text-gray-400 w-10">투명도</span>
        <input data-a="op" type="range" class="sm blue flex-1" min="0" max="100" value="${Math.round(r.opts.opacity * 100)}" />
        <span class="text-[10px] font-mono font-bold text-blue-600 w-8 text-right">${Math.round(r.opts.opacity * 100)}%</span>
      </div>
      <div data-a="panel" class="${r.showOpts ? "" : "hidden"} grid grid-cols-2 gap-1.5 pt-1 border-t border-gray-100">
        ${r.type === "drawing" ? `
        <label class="block"><span class="text-[10px] text-gray-500">방향</span>
          <select data-o="orient" class="inp"><option value="plan">평면 (수평)</option><option value="elev">입면/단면 (수직)</option></select></label>
        <label class="block"><span class="text-[10px] text-gray-500">표시 순서</span>
          <select data-o="order" class="inp"><option value="top">스캔 위에 겹침</option><option value="under">스캔 아래 깔기</option></select></label>
        <label class="flex items-center gap-1 text-[11px] text-gray-600"><input data-o="lineOnly" type="checkbox" class="accent-[#3B82F6]"> 흰 배경 제거</label>
        <label class="flex items-center gap-1 text-[11px] text-gray-600"><input data-o="useTint" type="checkbox" class="accent-[#3B82F6]"> 선 색상
          <input data-o="tint" type="color" class="w-6 h-5 ml-auto border-0 bg-transparent p-0"></label>` : `
        <label class="flex items-center gap-1 text-[11px] text-gray-600 col-span-1"><input data-o="ghost" type="checkbox" class="accent-[#3B82F6]"> 단색(고스트)</label>
        <label class="flex items-center gap-1 text-[11px] text-gray-600"><input data-o="color" type="color" class="w-6 h-5 border-0 bg-transparent p-0"> 고스트 색</label>`}
      </div>`;
    el.querySelector('[data-a="vis"]').onclick = () => { r.visible = !r.visible; applyVisibility(); renderAll(); };
    el.querySelector('[data-a="sel"]').onclick = () => { state.activeRef = r; renderAll(); };
    el.querySelector('[data-a="del"]').onclick = () => removeRef(r);
    el.querySelector('[data-a="opt"]').onclick = () => { r.showOpts = !r.showOpts; renderAll(); };
    // picking a colour turns on single-colour display (BIM) or line tint (drawing); the gear panel turns it off
    const col = el.querySelector('[data-a="col"]');
    col.oninput = () => {
      if (r.type === "drawing") { r.opts.tint = col.value; r.opts.useTint = true; }
      else { r.opts.color = col.value; r.opts.ghost = true; }
      styleRef(r);
    };
    col.onchange = () => renderAll();
    const op = el.querySelector('[data-a="op"]');
    op.oninput = () => { r.opts.opacity = op.value / 100; styleRef(r); op.nextElementSibling.textContent = op.value + "%"; };
    for (const inp of el.querySelectorAll("[data-o]")) {
      const key = inp.dataset.o;
      if (key === "orient") inp.value = r.orient;
      else if (inp.type === "checkbox") inp.checked = r.opts[key];
      else inp.value = r.opts[key];
      inp.oninput = inp.onchange = () => {
        if (key === "orient") r.orient = inp.value;
        else r.opts[key] = inp.type === "checkbox" ? inp.checked : inp.value;
        styleRef(r);
      };
    }
    list.appendChild(el);
  }
}

// top-left status pills
function renderPills() {
  const box = $("layer-pills");
  const pill = (dot, border, name, info, onX) => {
    const el = document.createElement("div");
    el.className = `flex items-center space-x-2 px-3 py-1.5 bg-white/90 backdrop-blur-md rounded-full shadow-md text-xs border ${border}`;
    el.innerHTML = `<span class="w-2 h-2 rounded-full ${dot}"></span><span class="font-bold text-gray-800 max-w-[180px] truncate">${esc(name)}</span>
      <span class="text-gray-400 text-[10px]">${esc(info)}</span><button class="text-gray-400 hover:text-red-500 ml-1"><i class="fa-solid fa-xmark"></i></button>`;
    el.querySelector("button").onclick = onX;
    box.appendChild(el);
  };
  box.innerHTML = "";
  const s = state.activeScan, r = state.activeRef;
  if (s) pill("bg-emerald-500 animate-pulse", "border-orange-200", s.name, s.count ? `${s.count.toLocaleString()} pts` : s.date, () => removeScan(s));
  if (r) pill("bg-blue-500", "border-blue-200", r.name, r.type === "bim" ? "BIM Overlay" : `${(KIND_BADGE[r.kind] || ["REF"])[0]} Layer`, () => removeRef(r));
  $("header-date").textContent = s ? s.date.replaceAll("-", ".") : "-";
}

function renderAll() {
  renderScans();
  renderRefs();
  renderPills();
  renderAlign();
  refreshEmpty();
}

$("input-ply").addEventListener("change", (e) => { onScanFiles([...e.target.files]); e.target.value = ""; });
$("btn-add-ply").addEventListener("click", () => $("input-ply").click());
$("btn-add-cad").addEventListener("click", () => $("input-cad").click());
$("input-cad").addEventListener("change", (e) => { const f = e.target.files[0]; e.target.value = ""; if (f) onRefFile(f); });
$("btn-add-bim").addEventListener("click", () => $("input-bim").click());
$("input-bim").addEventListener("change", (e) => { const f = e.target.files[0]; e.target.value = ""; if (f) onRefFile(f); });
for (const [zone, cb] of [["dropzone-ply", (fs) => onScanFiles(fs)], ["dropzone-cad", (fs) => fs[0] && onRefFile(fs[0])], ["dropzone-bim", (fs) => fs[0] && onRefFile(fs[0])]]) {
  const z = $(zone);
  z.addEventListener("dragover", (e) => { e.preventDefault(); z.classList.add("border-zzikbang"); });
  z.addEventListener("dragleave", () => z.classList.remove("border-zzikbang"));
  z.addEventListener("drop", (e) => { e.preventDefault(); z.classList.remove("border-zzikbang"); cb([...e.dataTransfer.files]); });
}

// header layer filter
document.querySelectorAll(".layer-tab").forEach((b) => b.addEventListener("click", () => {
  state.layer = b.dataset.layer;
  document.querySelectorAll(".layer-tab").forEach((o) => {
    const on = o === b;
    o.classList.toggle("bg-white", on); o.classList.toggle("shadow-xs", on); o.classList.toggle("font-bold", on); o.classList.toggle("text-gray-800", on);
  });
  applyVisibility();
}));

// global opacity sliders
$("slider-ply-opacity").addEventListener("input", (e) => {
  state.scanOpacity = e.target.value / 100;
  $("label-ply-opacity").textContent = e.target.value + "%";
  for (const s of state.scans) { s.opacity = state.scanOpacity; s.mesh.opacity = s.opacity; }
  renderScans();
});
$("slider-cad-opacity").addEventListener("input", (e) => {
  state.refOpacity = e.target.value / 100;
  $("label-cad-opacity").textContent = e.target.value + "%";
  for (const r of state.refs) { r.opts.opacity = state.refOpacity; styleRef(r); }
  renderRefs();
});

// panel open/close
$("toggle-right-panel").addEventListener("click", () => $("right-panel").classList.toggle("collapsed"));
$("close-right-panel").addEventListener("click", () => $("right-panel").classList.add("collapsed"));

// =====================================================================
// 5. View generation & section cut (splats via SplatEdit, BIM/drawings via clipping planes)
// =====================================================================
const view = { mode: "plan", h: 1.2, depth: 0.5, cut: false };
// plane SDF: local +z points down for the top cut (removes y > top) and up for the bottom cut
const cutEdit = new SplatEdit({ rgbaBlendMode: SplatEditRgbaBlendMode.MULTIPLY, softEdge: 0.005 });
const cutTop = new SplatEditSdf({ type: SplatEditSdfType.PLANE, opacity: 0, color: new THREE.Color(1, 1, 1) });
cutTop.rotation.x = Math.PI / 2;
const cutBottom = new SplatEditSdf({ type: SplatEditSdfType.PLANE, opacity: 0, color: new THREE.Color(1, 1, 1) });
cutBottom.rotation.x = -Math.PI / 2;
cutEdit.add(cutTop);
const clipTop = new THREE.Plane(new THREE.Vector3(0, -1, 0), 0);
const clipBottom = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const sectionBox = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)), new THREE.LineBasicMaterial({ color: 0xff6b00 }));
sectionBox.visible = false;
scene.add(sectionBox);

function cutRange() {
  if (!view.cut || view.mode === "elevation") return null;
  return view.mode === "section" ? [view.h - view.depth / 2, view.h + view.depth / 2] : [-Infinity, view.h];
}
function cutPlanes() {
  const r = cutRange();
  if (!r) return [];
  return Number.isFinite(r[0]) ? [clipTop, clipBottom] : [clipTop];
}
function applyCut() {
  const r = cutRange();
  if (r && !cutEdit.parent) scene.add(cutEdit);
  if (!r && cutEdit.parent) scene.remove(cutEdit);
  if (r) {
    cutTop.position.y = r[1];
    clipTop.constant = r[1];
    if (Number.isFinite(r[0])) { cutBottom.position.y = r[0]; clipBottom.constant = -r[0]; if (!cutBottom.parent) cutEdit.add(cutBottom); }
    else if (cutBottom.parent) cutEdit.remove(cutBottom);
  }
  // drawings are flat sheets at their own level, so only BIM geometry is clipped
  for (const ref of state.refs) if (ref.type === "bim") for (const m of [...ref.materials, ref.edgeMat].filter(Boolean)) { m.clippingPlanes = cutPlanes(); m.needsUpdate = true; }
  // orange wire box showing the kept volume
  sectionBox.visible = !!r;
  if (r) {
    const b = state.scans.some((s) => s.group.visible) ? scanUnionBox() : sceneBox();
    const lo = Number.isFinite(r[0]) ? r[0] : Math.min(b.min.y, 0), hi = r[1];
    sectionBox.position.set((b.min.x + b.max.x) / 2, (lo + hi) / 2, (b.min.z + b.max.z) / 2);
    sectionBox.scale.set(Math.max(0.1, b.max.x - b.min.x), Math.max(0.01, hi - lo), Math.max(0.1, b.max.z - b.min.z));
  }
}

const VIEW_DESC = {
  plan: "평면: 뷰 높이 위를 잘라내고 위에서 내려다봅니다 (도면 평면과 비교).",
  elevation: "입면: 정면에서 바라봅니다. 입면도는 도면 설정에서 방향을 '입면'으로 바꾸세요.",
  section: "단면: 뷰 높이를 중심으로 뷰 깊이만큼의 얇은 띠만 남겨 벽·기둥 위치를 확인합니다.",
};
function setView(mode, moveCamera = true) {
  view.mode = mode;
  if (mode === "section") view.cut = true;
  $("cut-on").checked = view.cut;
  document.querySelectorAll(".view-btn").forEach((b) => b.classList.toggle("active", b.dataset.view === mode));
  $("view-desc").textContent = VIEW_DESC[mode];
  applyCut();
  if (!moveCamera) return;
  if (mode === "elevation") sideView(); else topView();
}
document.querySelectorAll(".view-btn").forEach((b) => b.addEventListener("click", () => setView(b.dataset.view)));
$("cut-on").addEventListener("change", (e) => { view.cut = e.target.checked; applyCut(); });
function bindPair(rangeId, numId, key) {
  const set = (v) => {
    view[key] = parseFloat(v) || 0;
    $(rangeId).value = view[key];
    $(numId).value = view[key].toFixed(2);
    applyCut();
  };
  $(rangeId).addEventListener("input", (e) => set(e.target.value));
  $(numId).addEventListener("change", (e) => set(e.target.value));
  return set;
}
const setHeight = bindPair("range-view-height", "num-view-height", "h");
bindPair("range-view-depth", "num-view-depth", "depth");
$("btn-height-to-ref").addEventListener("click", () => {
  const y = state.activeRef?.type === "drawing" ? state.activeRef.group.position.y : 0;
  view.cut = true;
  setHeight(y + 1.2);
  setView(view.mode === "elevation" ? "plan" : view.mode);
});

function scanUnionBox() {
  const box = new THREE.Box3();
  for (const s of state.scans) if (s.group.visible) box.union(scanBox(s));
  return box;
}
function sceneBox() {
  const box = new THREE.Box3();
  for (const s of state.scans) if (s.group.visible) box.union(scanBox(s));
  for (const r of state.refs) if (r.group.visible) { r.group.updateMatrixWorld(true); box.expandByObject(r.type === "drawing" ? r.mesh : r.group); }
  if (box.isEmpty()) box.setFromCenterAndSize(new THREE.Vector3(), new THREE.Vector3(10, 3, 10));
  return box;
}
function fitView() {
  const b = sceneBox();
  const c = b.getCenter(new THREE.Vector3()), rad = Math.max(1, b.getSize(new THREE.Vector3()).length() / 2);
  controls.target.copy(c);
  flySpeed = Math.max(0.5, rad * 0.35);
  camera.position.copy(c).add(new THREE.Vector3(0.6, 0.7, 1).normalize().multiplyScalar(rad * 2.3));
  camera.near = Math.max(0.01, rad / 500);
  camera.far = rad * 60;
  camera.updateProjectionMatrix();
}
function topView() {
  const b = sceneBox();
  const c = b.getCenter(new THREE.Vector3()), size = b.getSize(new THREE.Vector3());
  const span = Math.max(size.x, size.z / camera.aspect * 1, 1) * 0.6;
  const h = span / Math.tan((camera.fov / 2) * DEG);
  const y = cutRange() ? Math.min(cutRange()[1], b.max.y) : b.max.y;
  controls.target.set(c.x, y - 0.01, c.z);
  camera.position.set(c.x, y + h, c.z + h * 1e-4);
}
function sideView() {
  const b = sceneBox();
  const c = b.getCenter(new THREE.Vector3()), size = b.getSize(new THREE.Vector3());
  const span = Math.max(size.x, size.y * camera.aspect, 1) * 0.6;
  const d = span / Math.tan((camera.fov / 2) * DEG);
  controls.target.copy(c);
  camera.position.set(c.x, c.y, b.max.z + d);
}
$("btn-reset-view").addEventListener("click", fitView);
$("btn-top-view").addEventListener("click", () => setView("plan"));
$("btn-side-view").addEventListener("click", () => setView("elevation"));

// =====================================================================
// 6. Picking (3D)
// =====================================================================
const raycaster = new THREE.Raycaster();
function rayFrom(ev) {
  const rc = canvas.getBoundingClientRect();
  raycaster.setFromCamera(new THREE.Vector2(((ev.clientX - rc.left) / rc.width) * 2 - 1, -((ev.clientY - rc.top) / rc.height) * 2 + 1), camera);
}
function inCut(p) {
  const r = cutRange();
  return !r || (p.y >= r[0] - 0.005 && p.y <= r[1] + 0.005);
}
function pickScan(ev) {
  rayFrom(ev);
  const visible = state.scans.filter((s) => s.group.visible);
  const hits = raycaster.intersectObjects(visible.map((s) => s.mesh), false)
    .filter((h) => inCut(h.point) && inCore(visible.find((s) => s.mesh === h.object), h.point));
  return hits[0]?.point.clone() || null;
}
function pickModel(ev, model) {
  rayFrom(ev);
  const meshes = model.group.children.filter((o) => o.isMesh);
  const hits = raycaster.intersectObjects(meshes, false).filter((h) => inCut(h.point));
  return hits[0]?.point.clone() || null;
}

function dot(color = 0xfacc15) {
  const m = new THREE.Mesh(new THREE.SphereGeometry(1, 12, 8), new THREE.MeshBasicMaterial({ color, depthTest: false }));
  m.renderOrder = 1000;
  m.onBeforeRender = () => m.scale.setScalar(Math.max(0.008, camera.position.distanceTo(m.position) * 0.005));
  return m;
}
// CSS2D labels keep their DOM node when an ancestor is removed, so drop them explicitly
function dropLabels(obj) {
  obj.traverse((o) => o.isCSS2DObject && o.element.remove());
}
function label(text, cls) {
  const d = document.createElement("div");
  d.className = cls;
  d.textContent = text;
  return new CSS2DObject(d);
}

// picking session: {labels, min, max, pickFn, pts, objs, resolve, allowSkip, text}
let pick = null;
function updateHint() {
  const h = $("hint");
  h.classList.toggle("hidden", !pick);
  if (!pick) return;
  const i = pick.pts.length;
  const next = i < pick.labels.length ? pick.labels[i] : "";
  $("hint-text").textContent = pick.text + (next && pick.max <= pick.labels.length ? ` — 지금: ${next}` : "");
  $("hint-count").textContent = pick.max === Infinity ? `${i}점` : `${i}/${pick.max}`;
  $("hint-skip").classList.toggle("hidden", !pick.allowSkip);
  $("hint-done").classList.toggle("hidden", !(pick.max !== pick.min && i >= pick.min));
}
function pick3D({ text, labels, min, max, pickFn, allowSkip = false, snapMarkers = false }) {
  cancelPick();
  return new Promise((resolve) => {
    pick = { text, labels, min, max, pickFn, allowSkip, snapMarkers, pts: [], objs: [], resolve };
    updateHint();
  });
}
function finishPick() {
  const { pts, objs, resolve } = pick;
  pick = null;
  updateHint();
  setTimeout(() => objs.forEach((o) => o && (helpers.remove(o), dropLabels(o))), 1500);
  resolve(pts);
}
function cancelPick() {
  if (!pick) return;
  pick.objs.forEach((o) => o && (helpers.remove(o), dropLabels(o)));
  const r = pick.resolve;
  pick = null;
  updateHint();
  r(null);
}
$("hint-cancel").onclick = cancelPick;
$("hint-done").onclick = () => pick && finishPick();
$("hint-undo").onclick = () => {
  if (!pick || !pick.pts.length) return;
  pick.pts.pop();
  const o = pick.objs.pop();
  if (o) { helpers.remove(o); dropLabels(o); }
  updateHint();
};
$("hint-skip").onclick = () => {
  if (!pick) return;
  pick.pts.push(null);
  pick.objs.push(null);
  updateHint();
  if (pick.pts.length >= pick.max) finishPick();
};

let downAt = null;
canvas.addEventListener("pointerdown", (e) => (downAt = { x: e.clientX, y: e.clientY }));
canvas.addEventListener("pointerup", (e) => {
  if (!downAt || e.button !== 0) return;
  const moved = Math.hypot(e.clientX - downAt.x, e.clientY - downAt.y);
  downAt = null;
  if (moved > 5) return;
  if (!pick) { if (dim.on) rulerClick(e); return; }
  const p = (pick.snapMarkers && markerNear(e, 22)) || pick.pickFn(e);
  if (!p) return toast("표면을 찾지 못했습니다 — 스캔(또는 모델) 위를 클릭하세요", 2000);
  const o = dot();
  o.position.copy(p);
  o.add(label(pick.labels[pick.pts.length] ?? String(pick.pts.length + 1), "pick3d"));
  helpers.add(o);
  pick.pts.push(p);
  pick.objs.push(o);
  updateHint();
  if (pick.pts.length >= pick.max) finishPick();
});
canvas.addEventListener("dblclick", (e) => {
  const p = pickScan(e);
  if (p) controls.target.copy(p);
});

// =====================================================================
// 7. Drawing point picker (2D modal with zoom / pan / loupe)
// =====================================================================
function pickOnDrawing(r, { title, sub, min, max, withLength, labels }) {
  return new Promise((resolve) => {
    const modal = $("modal-pick"), cv = $("mp-canvas"), wrap = $("mp-wrap"), lens = $("mp-lens");
    const ctx = cv.getContext("2d"), lctx = lens.getContext("2d");
    $("mp-title").textContent = title;
    $("mp-sub").textContent = sub;
    $("mp-len-wrap").style.display = withLength ? "flex" : "none";
    $("mp-len").value = "";
    modal.classList.remove("hidden");
    modal.classList.add("flex");
    const pts = [];
    const lab = (i) => labels?.[i] ?? LETTERS[i] ?? String(i + 1);
    const dpr = window.devicePixelRatio || 1;
    let W = 0, H = 0, scale = 1, ox = 0, oy = 0, hover = null, drag = null, fitted = false;
    // the backing store must track the element's real size, or clicks drift from what is drawn
    const sync = () => {
      const w = wrap.clientWidth, h = wrap.clientHeight;
      if (!w || !h) return;
      if (!fitted) {
        scale = Math.min(w / r.pxW, h / r.pxH) * 0.95;
        ox = (w - r.pxW * scale) / 2; oy = (h - r.pxH * scale) / 2;
        fitted = true;
      } else { ox += (w - W) / 2; oy += (h - H) / 2; }
      W = w; H = h;
      cv.width = W * dpr; cv.height = H * dpr;
      draw();
    };
    const ro = new ResizeObserver(sync);
    const toImg = (e) => { const b = cv.getBoundingClientRect(); return { x: (e.clientX - b.left - ox) / scale, y: (e.clientY - b.top - oy) / scale }; };
    // Shift locks horizontal/vertical to the previous point; otherwise snap to CAD endpoints or
    // raster corners within 14 screen px (Alt or the checkbox turns snapping off)
    const snap = (p, e) => {
      if (e.shiftKey && pts.length) {
        const a = pts.at(-1);
        return Math.abs(p.x - a.x) > Math.abs(p.y - a.y) ? { x: p.x, y: a.y } : { x: a.x, y: p.y };
      }
      if (e.altKey || !$("mp-snap").checked) return p;
      const radius = 14 / scale;
      const s = r.snap ? snapEndpoint(r.snap, p.x, p.y, radius) : snapCorner(r.canvas, p.x, p.y, radius);
      return s ? { x: s.x, y: s.y, snapped: s.kind } : p;
    };
    const okEnabled = () => pts.length >= min && (!withLength || +$("mp-len").value > 0);
    const draw = () => {
      if (!fitted) return;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, W, H);
      ctx.imageSmoothingEnabled = scale < 1;
      ctx.drawImage(r.canvas, ox, oy, r.pxW * scale, r.pxH * scale);
      const all = hover && pts.length < max ? [...pts, hover] : pts;
      if (withLength && all.length > 1) {
        ctx.strokeStyle = "#FF6B00"; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
        ctx.beginPath(); all.forEach((p, i) => ctx[i ? "lineTo" : "moveTo"](ox + p.x * scale, oy + p.y * scale)); ctx.stroke();
        ctx.setLineDash([]);
      }
      all.forEach((p, i) => {
        const x = ox + p.x * scale, y = oy + p.y * scale, fixed = i < pts.length;
        ctx.strokeStyle = fixed ? "#FF6B00" : "#2563eb"; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(x - 14, y); ctx.lineTo(x + 14, y); ctx.moveTo(x, y - 14); ctx.lineTo(x, y + 14); ctx.stroke();
        ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2); ctx.stroke();
        if (!fixed && p.snapped) {
          ctx.strokeStyle = "#16a34a"; ctx.lineWidth = 2.5;
          ctx.strokeRect(x - 8, y - 8, 16, 16);
          ctx.font = "bold 12px Pretendard, sans-serif";
          ctx.fillStyle = "#16a34a"; ctx.fillText(p.snapped, x + 12, y + 22);
        }
        if (fixed) {
          const t = lab(i);
          ctx.font = "bold 13px Pretendard, sans-serif";
          const w = ctx.measureText(t).width + 10;
          ctx.fillStyle = "#FF6B00"; ctx.fillRect(x + 9, y - 25, w, 18);
          ctx.fillStyle = "#fff"; ctx.fillText(t, x + 14, y - 12);
        }
      });
      let sub2 = sub;
      if (withLength && pts.length === 2) {
        const px = Math.hypot(pts[1].x - pts[0].x, pts[1].y - pts[0].y);
        sub2 = `두 점 간 ${px.toFixed(1)} px` + (r.calibrated ? ` (현재 축척으로 ${(px / r.ppm * 1000).toFixed(0)} mm)` : "") + " — 실제 치수를 입력하세요";
      }
      $("mp-sub").textContent = sub2;
      $("mp-count").textContent = max === Infinity ? `${pts.length}점 (최소 ${min})` : `${pts.length}/${max}`;
      $("mp-ok").disabled = !okEnabled();
    };
    const drawLens = (e) => {
      if (!hover || drag?.moved) { lens.classList.add("hidden"); return; }
      const b = wrap.getBoundingClientRect();
      lens.classList.remove("hidden");
      lens.style.left = `${Math.min(b.width - 160, e.clientX - b.left + 24)}px`;
      lens.style.top = `${Math.max(4, e.clientY - b.top - 174)}px`;
      const z = Math.max(scale * 4, 1.5); // loupe magnification in screen px per image px
      const src = 150 / z;
      lctx.fillStyle = "#fff"; lctx.fillRect(0, 0, 150, 150);
      lctx.imageSmoothingEnabled = false;
      lctx.drawImage(r.canvas, hover.x - src / 2, hover.y - src / 2, src, src, 0, 0, 150, 150);
      lctx.strokeStyle = "#FF6B00"; lctx.lineWidth = 1;
      lctx.beginPath(); lctx.moveTo(75, 0); lctx.lineTo(75, 150); lctx.moveTo(0, 75); lctx.lineTo(150, 75); lctx.stroke();
    };
    const onWheel = (e) => {
      e.preventDefault();
      const b = cv.getBoundingClientRect(), mx = e.clientX - b.left, my = e.clientY - b.top;
      const k = Math.exp(-e.deltaY * 0.0015);
      ox = mx - (mx - ox) * k; oy = my - (my - oy) * k; scale *= k;
      draw();
    };
    const onDown = (e) => { drag = { x: e.clientX, y: e.clientY, ox, oy, moved: false }; cv.setPointerCapture(e.pointerId); };
    const onMove = (e) => {
      if (drag) {
        const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
        if (Math.hypot(dx, dy) > 4) drag.moved = true;
        if (drag.moved) { ox = drag.ox + dx; oy = drag.oy + dy; }
      }
      hover = snap(toImg(e), e);
      draw();
      drawLens(e);
    };
    const onUp = (e) => {
      const click = drag && !drag.moved;
      drag = null;
      if (click && pts.length < max) {
        pts.push(snap(toImg(e), e));
        draw();
        if (pts.length === max && !withLength) close(true);
      }
    };
    const onLeave = () => { hover = null; lens.classList.add("hidden"); draw(); };
    const onKey = (e) => {
      if (e.key === "Escape") { e.stopPropagation(); close(false); }
      if (e.key === "Enter" && okEnabled()) close(true);
      if ((e.key === "z" && (e.ctrlKey || e.metaKey)) || e.key === "Backspace") { if (document.activeElement !== $("mp-len")) { pts.pop(); draw(); } }
    };
    const close = (ok) => {
      ro.disconnect();
      modal.classList.add("hidden");
      modal.classList.remove("flex");
      cv.removeEventListener("wheel", onWheel);
      cv.removeEventListener("pointerdown", onDown);
      cv.removeEventListener("pointermove", onMove);
      cv.removeEventListener("pointerup", onUp);
      cv.removeEventListener("pointerleave", onLeave);
      window.removeEventListener("keydown", onKey, true);
      resolve(ok ? { pts, length: +$("mp-len").value } : null);
    };
    cv.addEventListener("wheel", onWheel, { passive: false });
    cv.addEventListener("pointerdown", onDown);
    cv.addEventListener("pointermove", onMove);
    cv.addEventListener("pointerup", onUp);
    cv.addEventListener("pointerleave", onLeave);
    window.addEventListener("keydown", onKey, true);
    $("mp-undo").onclick = () => { pts.pop(); draw(); };
    $("mp-cancel").onclick = () => close(false);
    $("mp-ok").onclick = () => okEnabled() && close(true);
    $("mp-len").oninput = draw;
    $("mp-snap").onchange = draw;
    // image px -> client px, for scripted checks of the picker
    modal.__toScreen = (px, py) => { const b = cv.getBoundingClientRect(); return { x: b.left + ox + px * scale, y: b.top + oy + py * scale }; };
    ro.observe(wrap);
  });
}

// =====================================================================
// 8. Alignment panel
// =====================================================================
function openAlign(on) {
  $("panel-main").classList.toggle("hidden", on);
  $("panel-main").classList.toggle("flex", !on);
  $("panel-align").classList.toggle("hidden", !on);
  $("panel-align").classList.toggle("flex", on);
  $("right-panel").classList.remove("collapsed");
  if (on) renderAlign();
}
$("btn-sync-align").addEventListener("click", () => {
  if (!$("panel-align").classList.contains("hidden")) return openAlign(false);
  if (!state.scans.length && !state.refs.length) return toast("먼저 3DGS 스캔과 도면/BIM 파일을 업로드하세요");
  openAlign(true);
});
$("align-back").addEventListener("click", () => { cancelPick(); openAlign(false); });

$("al-scan").addEventListener("change", (e) => { state.activeScan = state.scans.find((s) => s.id === +e.target.value) || null; renderAll(); });
$("al-ref").addEventListener("change", (e) => { state.activeRef = state.refs.find((r) => r.id === +e.target.value) || null; renderAll(); });

let lastAlignScanId = null;
function renderAlign() {
  // title = the files being aligned, so switching drawing / BIM is obvious
  {
    const sc = state.activeScan, rf = state.activeRef;
    const tag = !rf ? "" : rf.type === "bim"
      ? `<span class="text-[9px] font-extrabold px-1 rounded bg-emerald-50 text-emerald-600 mr-1">BIM</span>`
      : `<span class="text-[9px] font-extrabold px-1 rounded bg-blue-50 text-blue-600 mr-1">도면</span>`;
    $("al-title").innerHTML = sc || rf
      ? `${tag}${esc(rf?.name || "기준 없음")} <span class="text-gray-300 font-normal">↔</span> <span class="text-zzikbang">${esc(sc?.name || "스캔 없음")}</span>`
      : "스캔본 & 도면 위치 정렬";
    $("al-title").title = `${rf?.name || "-"} ↔ ${sc?.name || "-"}`;
  }
  const ls = state.activeScan?.locked;
  $("al-lock").innerHTML = `<i class="fa-solid fa-${ls ? "lock" : "lock-open"}"></i><span>${ls ? "고정됨" : "고정"}</span>`;
  $("al-lock").className = `shrink-0 flex items-center gap-1 px-2 py-1 rounded-md text-[11px] font-bold border ${ls ? "bg-zzikbang text-white border-zzikbang" : "bg-white text-gray-500 border-gray-200 hover:text-zzikbang"}`;
  $("al-lock").title = ls ? "정렬 고정 해제" : "지금 정렬 상태를 고정 (스캔과 맞춰 둔 도면/BIM이 더 이상 움직이지 않음)";
  const opt = (arr, active, none) => arr.length ? arr.map((o) => `<option value="${o.id}" ${o === active ? "selected" : ""}>${esc(o.name)}</option>`).join("") : `<option>${none}</option>`;
  $("al-scan").innerHTML = opt(state.scans, state.activeScan, "스캔 없음");
  $("al-ref").innerHTML = opt(state.refs, state.activeRef, "도면/BIM 없음");
  const r = state.activeRef;
  // step 2 only applies to raster drawings without known scale
  const isDrawing = r?.type === "drawing";
  $("al-step-scale").classList.toggle("hidden", !isDrawing);
  const badge = $("al-calib-badge");
  if (!r) badge.textContent = "";
  else if (r.type === "bim") { badge.textContent = "BIM 실척"; badge.className = "ml-auto text-[10px] font-bold px-1.5 py-0.5 rounded bg-emerald-50 text-emerald-600"; }
  else if (r.calibrated) { badge.textContent = "보정 완료"; badge.className = "ml-auto text-[10px] font-bold px-1.5 py-0.5 rounded bg-emerald-50 text-emerald-600"; }
  else { badge.textContent = "미보정"; badge.className = "ml-auto text-[10px] font-bold px-1.5 py-0.5 rounded bg-amber-50 text-amber-600"; }
  if (isDrawing) {
    if (document.activeElement !== $("dwg-height")) $("dwg-height").value = r.group.position.y.toFixed(2);
    if (r.dpi) $("al-dpi").value = Math.round(r.dpi);
    $("al-scale-info").textContent = `${r.pxW}×${r.pxH}px · 1 m = ${r.ppm.toFixed(2)} px · 도면 실제 크기 ${(r.pxW / r.ppm).toFixed(2)} × ${(r.pxH / r.ppm).toFixed(2)} m` +
      (r.kind === "dwg" || r.kind === "dxf" ? " (CAD 단위로 자동 설정됨)" : "");
  } else $("al-scale-info").textContent = r ? "IFC 모델은 단위 정보로 실척 배치되므로 축척 보정이 필요 없습니다." : "";
  // LiDAR scans are already metric: default to rotation + translation only when a new scan is selected
  if (state.activeScan && state.activeScan.id !== lastAlignScanId) {
    lastAlignScanId = state.activeScan.id;
    if (state.activeScan.metric) $("al-scale").value = "0";
  }
  const rr = state.activeRef;
  // what moves: a 2D drawing stays put (the scan moves) unless the scan already has another anchor,
  // in which case this reference moves to the scan
  const sc0 = state.activeScan, anchor = anchorOf(sc0);
  const forced = sc0 && rr && (rr.type === "drawing" || (anchor && anchor !== rr)) ? moveTarget(sc0, rr) : null;
  const fixedRef = forced === "scan" && rr?.type === "drawing";
  if (forced) { $("al-target").value = forced; $("nudge-target").value = forced; }
  $("al-target").disabled = !!forced;
  $("nudge-target").disabled = !!forced;
  syncSegs();
  $("anchor-box").classList.toggle("hidden", !anchor);
  if (anchor) {
    $("anchor-note").textContent = anchor === rr ? `앵커 · ${anchor.name}` : `앵커 · ${anchor.name} → 이 기준이 이동`;
    $("anchor-box").title = anchor === rr
      ? `이 스캔의 앵커: ${anchor.name} — 스캔이 움직이면 함께 맞춰 둔 다른 기준도 같이 움직입니다.`
      : `이 스캔은 '${anchor.name}'(앵커)에 맞춰져 있습니다. 이번 기준 '${rr?.name ?? ""}'은 스캔에 맞춰 움직입니다.`;
  }
  $("xform-box").classList.toggle("hidden", fixedRef);
  $("al-level-wrap").classList.toggle("hidden", rr?.type !== "bim");
  if (rr?.type === "bim") {
    const lowest = rr.levels[0]?.y ?? 0;
    $("al-bimlevel").innerHTML = rr.levels.length
      ? rr.levels.map((l, i) => `<option value="${i}" ${i === rr.levelIdx ? "selected" : ""}>${l.y - lowest >= 0 ? "+" : ""}${(l.y - lowest).toFixed(2)} m · 바닥 면적 ${l.area.toFixed(0)} m²</option>`).join("")
      : "<option>수평면을 찾지 못함 (모델 최저점 사용)</option>";
  }
  $("mk-ifc-wrap").classList.toggle("hidden", rr?.type !== "bim");
  const dwgMarks = rr?.type === "drawing" && rr.refPts.length >= 2 && rr.refPts.every((p) => p.marker);
  $("mk-dwg-wrap").classList.toggle("hidden", !(rr?.type === "drawing" && rr.orient === "plan"));
  $("mk-dwg-label").textContent = dwgMarks ? `도면 마커 ${rr.refPts.length}개로 자동 정렬` : "도면에 마커 찍고 자동 정렬";
  $("mk-dwg-repick").classList.toggle("hidden", !dwgMarks);
  $("mk-export-label").textContent = rr?.type === "bim" && rr.kind !== "ifc" ? "IFC 변환·센서 저장" : "IFC에 마커 추가·저장";
  if (rr?.type === "bim") {
    // name-confirmed marker groups first, then objects found by the marker colour, then the rest
    const [tol, sat] = MARKER_SENS[$("mk-sens").value];
    const colored = colorMarkers(rr.pieces, { hue: markerHue(), hueTol: tol, minSat: sat });
    const colorName = $("mk-color").selectedOptions[0]?.textContent.split(" ")[0] || "색";
    const colorGroup = colored.length >= 2 ? [{ stem: `${colorName} 객체`, type: "색으로 찾음", items: colored, marker: true }] : [];
    rr.markerOptions = [...rr.markerGroups.filter((g) => g.marker), ...colorGroup, ...rr.markerGroups.filter((g) => !g.marker)];
    const gs = rr.markerOptions;
    const keep = $("mk-ifc-group").value;
    $("mk-ifc-group").innerHTML = gs.length
      ? gs.map((g, i) => `<option value="${i}" ${String(i) === keep ? "selected" : ""}>${g.marker ? "★ " : ""}${esc(g.stem)} · ${esc(g.type.replace(/^IFC/, "Ifc"))} (${g.items.length}개)</option>`).join("")
      : "<option>작은 객체 그룹을 찾지 못함</option>";
  }
  const sc = state.activeScan;
  $("mk-info").textContent = sc?.markers ? (sc.markers.length ? `인식된 마커 ${sc.markers.length}개 (M1 ~ M${sc.markers.length})` : "인식된 마커 없음") : "";
  const others = state.scans.filter((o) => o !== sc);
  const keep = $("mk-base").value;
  $("mk-base").innerHTML = others.length ? others.map((o) => `<option value="${o.id}" ${String(o.id) === keep ? "selected" : ""}>${esc(o.date)} · ${esc(o.name)}</option>`).join("") : "<option>다른 스캔 없음</option>";
  const fs = state.activeScan?.floor;
  // no status line: the floor button shows a check and keeps the details in its tooltip
  $("al-floor").title = fs ? `✓ ${fs.info}` : "";
  $("al-floor").innerHTML = `<i class="fa-solid fa-${fs ? "circle-check" : "layer-group"}"></i>바닥 인식`;
  $("al-floor-next").disabled = !(fs && fs.cands.length > 1);
  $("al-floor-undo").disabled = !fs;
  renderRefPts();
  updateXform();
}

// reference points as compact numbered chips; clicking one opens its editor (name, re-pick, delete)
let refPtOpen = -1;
function renderRefPts() {
  const r = state.activeRef, box = $("al-refpts");
  box.innerHTML = "";
  $("al-setref-label").textContent = r?.refPts.length ? `기준점 다시 지정 (${r.refPts.length}점)` : "기준점 지정";
  $("al-run").disabled = !(r && r.refPts.length >= 2 && state.activeScan);
  if (!r?.refPts.length) return;
  if (refPtOpen >= r.refPts.length) refPtOpen = -1;
  const chips = document.createElement("div");
  chips.className = "flex flex-wrap gap-1";
  r.refPts.forEach((p, i) => {
    const bad = p.residual != null && p.residual > 0.05;
    const c = document.createElement("button");
    c.className = `h-6 min-w-6 px-1.5 rounded-md text-[10px] font-extrabold border ${i === refPtOpen ? "bg-zzikbang text-white border-zzikbang" : "bg-white text-gray-700 border-gray-200 hover:border-zzikbang"}`;
    c.title = `${p.label}${p.residual != null ? ` · 오차 ${(p.residual * 1000).toFixed(0)} mm` : ""} — 눌러서 수정`;
    c.innerHTML = `${i + 1}${p.residual != null ? `<span class="inline-block w-1.5 h-1.5 rounded-full ml-1 align-middle ${bad ? "bg-red-500" : "bg-emerald-500"}"></span>` : ""}`;
    c.onclick = () => { refPtOpen = refPtOpen === i ? -1 : i; renderRefPts(); };
    chips.appendChild(c);
  });
  box.appendChild(chips);
  if (refPtOpen < 0) return;
  const i = refPtOpen, p = r.refPts[i];
  const row = document.createElement("div");
  row.className = "flex items-center gap-1 mt-1";
  const res = p.residual != null ? `<span class="text-[10px] font-mono ${p.residual > 0.05 ? "text-red-500" : "text-emerald-600"}">${(p.residual * 1000).toFixed(0)}mm</span>` : "";
  row.innerHTML = `<input class="inp flex-1" value="${esc(p.label)}" title="기준점 이름 (예: X1-Y1)" />${res}
    <button data-a="move" class="btn-sub px-2" title="위치 다시 찍기"><i class="fa-solid fa-location-crosshairs"></i></button>
    <button data-a="del" class="btn-sub px-2 hover:text-red-500" title="삭제"><i class="fa-solid fa-trash-can"></i></button>`;
  row.querySelector("input").onchange = (e) => { p.label = e.target.value || LETTERS[i]; rebuildPins(r); renderRefPts(); };
  row.querySelector('[data-a="del"]').onclick = () => { r.refPts.splice(i, 1); refPtOpen = -1; rebuildPins(r); renderRefPts(); };
  row.querySelector('[data-a="move"]').onclick = () => repickRefPt(r, i);
  box.appendChild(row);
}
async function repickRefPt(r, i) {
  const p = r.refPts[i];
  if (r.type === "drawing") {
    const res = await pickOnDrawing(r, { title: `기준점 ${i + 1} (${p.label}) 위치 수정`, sub: "새 위치를 클릭 후 확인", min: 1, max: 1, labels: [p.label] });
    if (!res) return;
    p.px = res.pts[0].x; p.py = res.pts[0].y;
  } else {
    const pts = await pick3D({ text: `기준점 ${i + 1} (${p.label})의 새 위치를 모델에서 클릭`, labels: [p.label], min: 1, max: 1, pickFn: (e) => pickModel(e, r) });
    if (!pts) return;
    r.group.updateMatrixWorld(true);
    p.local = pts[0].clone().applyMatrix4(r.group.matrixWorld.clone().invert()).toArray();
  }
  p.residual = null;
  rebuildPins(r);
  renderRefPts();
  toast(`기준점 ${i + 1} 위치를 수정했습니다`);
}

function rebuildPins(r) {
  dropLabels(r.pins);
  r.pins.clear();
  if (!$("al-showref").checked) return;
  for (const p of r.refPts) {
    const o = new THREE.Object3D();
    o.position.copy(r.type === "drawing" ? pixelToLocal(r, p.px, p.py) : new THREE.Vector3().fromArray(p.local));
    o.add(label(p.label, "ref3d"));
    r.pins.add(o);
  }
}
$("al-showref").addEventListener("change", () => state.refs.forEach(rebuildPins));

// ---- step 1: scan frame fixes
$("al-floor").onclick = () => {
  const s = state.activeScan; if (!s) return toast("스캔을 먼저 올려주세요");
  runFloor(s, $("al-floor-mode").value);
};
$("al-floor-next").onclick = () => {
  const s = state.activeScan; if (!s?.floor) return;
  applyFloorCandidate(s, (s.floor.idx + 1) % s.floor.cands.length);
};
$("al-floor-undo").onclick = () => {
  const s = state.activeScan; if (!s?.floor) return;
  restoreMatrix(s.group, s.floor.base);
  s.floor = null;
  s.floorLocal = null; // manual corrections invalidate the remembered floor plane
  renderAll();
  fitView();
  toast("바닥 인식을 되돌렸습니다");
};
$("al-flip").onclick = () => {
  const s = state.activeScan; if (!s) return toast("스캔을 먼저 올려주세요");
  applyWorld(s.group, aboutPoint(scanCenter(s), new THREE.Matrix4().makeRotationX(Math.PI)));
  s.floor = null;
  s.floorLocal = null; // manual corrections invalidate the remembered floor plane
  renderAll();
  fitView();
};
$("al-zup").onclick = () => {
  const s = state.activeScan; if (!s) return toast("스캔을 먼저 올려주세요");
  applyWorld(s.group, aboutPoint(scanCenter(s), new THREE.Matrix4().makeRotationX(-Math.PI / 2)));
  s.floor = null;
  s.floorLocal = null; // manual corrections invalidate the remembered floor plane
  renderAll();
  fitView();
};
$("al-level").onclick = async () => {
  const s = state.activeScan; if (!s) return toast("스캔을 먼저 올려주세요");
  const pts = await pick3D({ text: "스캔 바닥면에서 서로 멀리 떨어진 3점을 클릭하세요", labels: ["1", "2", "3"], min: 3, max: 3, pickFn: pickScan });
  if (!pts) return;
  const n = new THREE.Vector3().subVectors(pts[1], pts[0]).cross(new THREE.Vector3().subVectors(pts[2], pts[0])).normalize();
  if (n.y < 0) n.negate();
  const c = pts[0].clone().add(pts[1]).add(pts[2]).divideScalar(3);
  const M = aboutPoint(c, new THREE.Matrix4().makeRotationFromQuaternion(new THREE.Quaternion().setFromUnitVectors(n, UP)));
  M.premultiply(new THREE.Matrix4().makeTranslation(0, -c.y, 0)); // floor -> y = 0
  applyWorld(s.group, M);
  s.floor = null;
  s.floorLocal = null; // manual corrections invalidate the remembered floor plane
  renderAll();
  toast(`수평 보정 완료 (기울기 ${(Math.acos(Math.min(1, n.y)) / DEG).toFixed(2)}°) · 바닥 = 0 m`);
};
$("al-scanscale").onclick = async () => {
  const s = state.activeScan; if (!s) return toast("스캔을 먼저 올려주세요");
  const pts = await pick3D({ text: "실제 거리를 아는 두 지점을 스캔에서 클릭하세요", labels: ["1", "2"], min: 2, max: 2, pickFn: pickScan });
  if (!pts) return;
  const cur = pts[0].distanceTo(pts[1]);
  const v = await ask("스캔 실측 스케일", `스캔상 거리: ${cur.toFixed(4)} (단위 미상)`, [{ label: "실제 거리 (mm)", value: 1000 }]);
  const real = v && parseFloat(v[0]) / 1000;
  if (!(real > 0)) return;
  const k = real / cur;
  applyWorld(s.group, aboutPoint(pts[0], new THREE.Matrix4().makeScale(k, k, k)));
  toast(`스캔 스케일 ×${k.toFixed(4)} 적용`);
  fitView();
};

// ---- step 2: drawing scale
function setPpm(r, ppm) {
  // rescales about the drawing centre; reference points are stored in pixels so they stay on their features
  r.ppm = ppm;
  r.calibrated = true;
  rebuildDrawing(r);
  renderAll();
}
$("al-calib2").onclick = async () => {
  const r = state.activeRef; if (r?.type !== "drawing") return;
  const res = await pickOnDrawing(r, { title: "도면 축척 보정", sub: "치수를 알고 있는 두 점(치수선 양 끝, 기둥 중심 간 등)을 클릭하세요", min: 2, max: 2, withLength: true, labels: ["1", "2"] });
  if (!res) return;
  const px = Math.hypot(res.pts[1].x - res.pts[0].x, res.pts[1].y - res.pts[0].y);
  setPpm(r, px / (res.length / 1000));
  toast(`축척 보정 완료: ${px.toFixed(1)} px = ${res.length.toLocaleString()} mm`);
};
// drawing sheet height (the plan's level in 3D)
function setDrawingHeight(y) {
  const r = state.activeRef; if (r?.type !== "drawing" || !Number.isFinite(y)) return;
  if (lockOwner(r.group)) return lockedToast();
  r.group.position.y = y;
  r.group.updateMatrixWorld(true);
  $("dwg-height").value = y.toFixed(2);
  updateXform();
}
$("dwg-height").addEventListener("change", () => setDrawingHeight(+$("dwg-height").value));
$("dwg-height-up").onclick = (e) => setDrawingHeight((state.activeRef?.group.position.y ?? 0) + (e.shiftKey ? 0.01 : 0.1));
$("dwg-height-down").onclick = (e) => setDrawingHeight((state.activeRef?.group.position.y ?? 0) - (e.shiftKey ? 0.01 : 0.1));
$("dwg-height-floor").onclick = () => {
  const s = state.activeScan; if (!s) return toast("스캔을 먼저 올려주세요");
  setDrawingHeight(scanFloorY(s));
};
$("al-ratio-apply").onclick = () => {
  const r = state.activeRef; if (r?.type !== "drawing") return;
  const n = +$("al-ratio").value, dpi = +$("al-dpi").value;
  if (!(n > 0 && dpi > 0)) return;
  setPpm(r, dpi / 0.0254 / n);
  toast(`축척 1:${n} @ ${dpi} DPI 적용`);
};

// ---- step 3: reference points & automatic registration
$("al-setref").onclick = async () => {
  const r = state.activeRef; if (!r) return toast("도면 또는 BIM 파일을 먼저 올려주세요");
  const keep = r.refPts.map((p) => p.label);
  if (r.type === "drawing") {
    const res = await pickOnDrawing(r, {
      title: "기준점 지정 — 도면", min: 2, max: Infinity, labels: keep,
      sub: "스캔에서도 잘 보이는 점(기둥 중심, 벽 모서리, 개구부 끝 등)을 2개 이상 클릭 후 확인. 서로 멀리, 넓게 분포할수록 정확합니다.",
    });
    if (!res) return;
    r.refPts = res.pts.map((p, i) => ({ label: keep[i] || LETTERS[i], px: p.x, py: p.y }));
  } else {
    const pts = await pick3D({ text: "BIM 모델에서 기준점을 2개 이상 클릭 후 '완료'", labels: keep.length ? keep : [...LETTERS], min: 2, max: Infinity, pickFn: (e) => pickModel(e, r) });
    if (!pts) return;
    r.group.updateMatrixWorld(true);
    const inv = r.group.matrixWorld.clone().invert();
    r.refPts = pts.map((p, i) => ({ label: keep[i] || LETTERS[i], local: p.clone().applyMatrix4(inv).toArray() }));
  }
  rebuildPins(r);
  renderRefPts();
  toast(`기준점 ${r.refPts.length}개 저장 — 이제 '스캔에서 대응점 찍고 자동 정합'을 누르세요`);
};

/**
 * Least-squares similarity constrained to rotations about the vertical axis (scans and drawings
 * share "up"). planar=true fits only X/Z (plan drawings carry no height information).
 */
function solveYawSimilarity(P, Q, { withScale, planar }) {
  const n = P.length;
  const pc = new THREE.Vector3(), qc = new THREE.Vector3();
  P.forEach((p) => pc.add(p)); Q.forEach((q) => qc.add(q));
  pc.divideScalar(n); qc.divideScalar(n);
  let dotS = 0, crossS = 0;
  const A = P.map((p) => p.clone().sub(pc)), B = Q.map((q) => q.clone().sub(qc));
  if (planar) { A.forEach((a) => (a.y = 0)); B.forEach((b) => (b.y = 0)); }
  // rotation.y = θ rotates w = (x, -z) by +θ, so solve the 2D Procrustes problem on w
  for (let i = 0; i < n; i++) {
    const ax = A[i].x, ay = -A[i].z, bx = B[i].x, by = -B[i].z;
    dotS += ax * bx + ay * by;
    crossS += ax * by - ay * bx;
  }
  const theta = Math.atan2(crossS, dotS);
  const R = new THREE.Matrix4().makeRotationY(theta);
  let s = 1;
  if (withScale) {
    let num = 0, den = 0;
    for (let i = 0; i < n; i++) { num += B[i].dot(A[i].clone().applyMatrix4(R)); den += A[i].lengthSq(); }
    s = den > 1e-12 ? num / den : 1;
  }
  const RS = R.clone().multiply(new THREE.Matrix4().makeScale(s, s, s));
  const t = qc.clone().sub(pc.clone().applyMatrix4(RS));
  if (planar) t.y = 0;
  const M = new THREE.Matrix4().makeTranslation(t.x, t.y, t.z).multiply(RS);
  const residuals = P.map((p, i) => {
    const d = p.clone().applyMatrix4(M).sub(Q[i]);
    if (planar) d.y = 0;
    return d.length();
  });
  return { M, s, theta, residuals, rms: Math.sqrt(residuals.reduce((a, r) => a + r * r, 0) / n) };
}

// two-choice selects shown as compact toggle buttons; the hidden <select> stays the source of truth
var SEGS = ["al-target", "al-scale", "nudge-target"]; // var: renderAlign may run before this line
for (const id of SEGS) {
  const sel = $(id);
  const seg = document.createElement("div");
  seg.id = id + "-seg";
  seg.className = "grid grid-cols-2 gap-0.5 bg-gray-100 p-0.5 rounded-md text-[11px] font-semibold mt-0.5";
  for (const o of sel.options) {
    const b = document.createElement("button");
    b.type = "button";
    b.dataset.v = o.value;
    b.textContent = o.textContent;
    b.className = "py-1 rounded";
    b.onclick = () => { if (sel.disabled || sel.value === o.value) return; sel.value = o.value; sel.dispatchEvent(new Event("change")); syncSegs(); };
    seg.appendChild(b);
  }
  sel.classList.add("hidden");
  sel.after(seg);
}
function syncSegs() {
  for (const id of SEGS || []) {
    const sel = $(id), seg = $(id + "-seg");
    if (!seg) continue;
    for (const b of seg.children) {
      const on = b.dataset.v === sel.value;
      b.classList.toggle("bg-white", on); b.classList.toggle("shadow-xs", on); b.classList.toggle("text-gray-800", on); b.classList.toggle("text-gray-400", !on);
      b.disabled = sel.disabled;
      b.classList.toggle("cursor-not-allowed", sel.disabled);
    }
  }
}
syncSegs();
// align steps fold by clicking their header (all start folded)
document.querySelectorAll("#panel-align .step-num").forEach((num) => {
  const head = num.parentElement, step = head.parentElement;
  head.classList.add("step-head");
  const chev = document.createElement("i");
  chev.className = "fa-solid fa-chevron-down step-chev text-[10px] text-gray-300 transition-transform";
  head.appendChild(chev);
  head.addEventListener("click", (e) => { if (!e.target.closest("button, input, select")) step.classList.toggle("step-collapsed"); });
  step.classList.add("step-collapsed");
});
$("al-lock").onclick = () => {
  const s = state.activeScan; if (!s) return toast("스캔을 먼저 올려주세요");
  s.locked = !s.locked;
  renderAlign();
  toast(s.locked ? `정렬을 고정했습니다 — ${s.name}${s.linked?.size ? ` + 기준 ${s.linked.size}개` : ""}` : "정렬 고정을 풀었습니다");
};
$("al-run").onclick = async () => {
  const r = state.activeRef, s = state.activeScan;
  if (!r || !s || r.refPts.length < 2) return;
  if (blockedByLock(s, r)) return;
  const withScale = $("al-scale").value === "1";
  const moveScan = moveTarget(s, r) === "scan";
  if (r.type === "drawing" && !r.calibrated && moveScan && withScale) {
    const ok = await ask("도면 축척이 보정되지 않았습니다", "이대로 스캔을 도면에 맞추면 스캔 치수가 틀어집니다. 2단계에서 축척을 먼저 맞추는 것을 권장합니다.\n\n그래도 진행할까요?");
    if (!ok) return;
  }
  const labels = r.refPts.map((p) => p.label);
  const picked = await pick3D({ text: "스캔에서 기준점과 같은 지점을 순서대로 클릭 (안 보이면 건너뛰기, 마커 근처는 중심에 붙음)", labels, min: labels.length, max: labels.length, pickFn: pickScan, allowSkip: true, snapMarkers: true });
  if (!picked) return;
  const pairs = picked.map((q, i) => q && { ref: refPointWorld(r, r.refPts[i]), scan: q, i }).filter(Boolean);
  if (pairs.length < 2) return toast("대응점이 2개 이상 필요합니다");
  const planar = isPlanarRef(r);
  const refW = pairs.map((p) => p.ref), scanW = pairs.map((p) => p.scan);
  let res;
  if (moveScan) {
    res = solveYawSimilarity(scanW, refW, { withScale, planar });
    moveScanWith(s, res.M, r);
  } else {
    res = solveYawSimilarity(refW, scanW, { withScale, planar });
    applyWorld(r.group, res.M);
    r.group.quaternion.setFromAxisAngle(UP, yawOf(r.group.quaternion)); // stay level
  }
  if (planar) seatFloor(s, r, moveScan);
  linkRef(s, r);
  r.refPts.forEach((p) => (p.residual = null));
  pairs.forEach((p, k) => (r.refPts[p.i].residual = res.residuals[k]));
  const box = $("al-result");
  box.classList.remove("hidden");
  const quality = res.rms < 0.02 ? ["매우 정확", "text-emerald-600"] : res.rms < 0.05 ? ["양호", "text-emerald-600"] : res.rms < 0.15 ? ["확인 필요", "text-amber-600"] : ["점 대응 오류 의심", "text-red-500"];
  box.innerHTML = `
    <p class="font-bold text-gray-900"><i class="fa-solid fa-circle-check text-zzikbang mr-1"></i>자동 정합 완료 — ${pairs.length}점 사용</p>
    <p>회전 ${(res.theta / DEG).toFixed(2)}° · 배율 ×${res.s.toFixed(4)}</p>
    <p>${pairs.length > 2 || !withScale ? `평균 오차(RMS) <b class="${quality[1]}">${(res.rms * 1000).toFixed(0)} mm · ${quality[0]}</b>` : "2점 정합은 오차를 검증할 수 없습니다 — 3점 이상을 권장합니다"}</p>
    ${res.residuals.some((x) => x > Math.max(0.05, res.rms * 2)) ? `<p class="text-red-500">오차가 큰 점이 있습니다. 순서가 맞는지 확인하거나 해당 점을 삭제 후 다시 실행하세요.</p>` : ""}`;
  s.aligned = true;
  renderRefPts();
  updateXform();
  fitView();
  // clicked points are rarely exact: tighten the fit against the wall section
  if ($("al-autorefine").checked && autoSupported(r)) await runRefine();
};

// ---- automatic alignment: wall-height slice of the scan vs. plan lines / BIM section
function autoSupported(r) { return !!r && (r.type === "bim" || r.orient === "plan"); }
// plans and BIM are matched in plan (x/z) only; height always comes from floor-to-level seating,
// so points picked on roofs or ceilings cannot lift the scan floor
const isPlanarRef = (r) => r.type === "bim" || (r.type === "drawing" && r.orient === "plan");
// ---- anchors: the first reference a scan is aligned to stays put; references aligned later move to the
// scan, and when the scan itself moves every other reference already aligned to it moves along
function linkedRefs(s) { return state.refs.filter((o) => s.linked?.has(o.id)); }
function anchorOf(s) { return state.refs.find((o) => o.id === s?.anchor) || null; }
function moveTarget(s, r) {
  if (s?.locked) return "ref"; // a locked scan stays put; new references come to it
  const anchor = anchorOf(s);
  if (anchor && anchor !== r) return "ref";
  return r.type === "drawing" ? "scan" : $("al-target").value;
}
function moveScanWith(s, M, r) {
  applyWorld(s.group, M);
  for (const o of linkedRefs(s)) {
    if (o === r) continue;
    applyWorld(o.group, M);
    o.group.quaternion.setFromAxisAngle(UP, yawOf(o.group.quaternion)); // references stay level
  }
}
function linkRef(s, r) {
  s.linked ??= new Set();
  s.linked.add(r.id);
  if (!anchorOf(s)) s.anchor = r.id;
  renderAlign(); // the anchor note and the locked "what moves" choice change now
}
function seatFloor(s, r, moveScan) {
  const floor = scanFloorY(s), level = refLevel(r);
  if (moveScan) moveScanWith(s, new THREE.Matrix4().makeTranslation(0, level - floor, 0), r);
  else if (r.type === "drawing") r.group.position.y = floor;
  else applyWorld(r.group, new THREE.Matrix4().makeTranslation(0, floor - level, 0));
}
// height of the reference's floor in world space: the drawing sheet, or the chosen BIM floor level
function refLevel(r) {
  if (r.type === "drawing") return r.group.position.y;
  r.group.updateMatrixWorld(true);
  const lv = r.levels?.[r.levelIdx];
  if (lv) return new THREE.Vector3(0, lv.y, 0).applyMatrix4(r.group.matrixWorld).y;
  const b = new THREE.Box3();
  for (const m of r.group.children) if (m.isMesh) b.expandByObject(m);
  return b.min.y;
}
// height of the scan's floor in world space: the detected floor plane when known
function scanFloorY(s) {
  s.group.updateMatrixWorld(true);
  if (s.floorLocal) return s.floorLocal.clone().applyMatrix4(s.mesh.matrixWorld).y;
  let P = sampleSplats(s.mesh, s.count, 100_000);
  if (s.core) P = filterToBox(P, s.core.box, s.mesh.matrixWorld);
  const ys = [];
  for (let i = 1; i < P.length; i += 3) ys.push(P[i]);
  ys.sort((a, b) => a - b);
  return ys[Math.floor(ys.length * 0.02)] ?? 0;
}
function scanSlice(s) {
  s.group.updateMatrixWorld(true);
  let P = sampleSplats(s.mesh, s.count, 400_000);
  if (s.core) P = filterToBox(P, s.core.box, s.mesh.matrixWorld);
  const ys = [];
  for (let i = 1; i < P.length; i += 3) ys.push(P[i]);
  ys.sort((a, b) => a - b);
  const floor = s.floorLocal ? scanFloorY(s) : ys[Math.floor(ys.length * 0.02)], top = ys[Math.floor(ys.length * 0.98)];
  // metres only mean something once the scan is metric (LiDAR) or has been scaled to the drawing
  const [y0, y1] = s.metric || s.aligned
    ? [floor + +$("al-slice0").value, floor + +$("al-slice1").value]
    : [floor + (top - floor) * 0.32, floor + (top - floor) * 0.57];
  return { floor, S: sliceScan(P, y0, y1) };
}
function refField(r, level) {
  if (r.type === "drawing") return drawingField(r);
  const f = bimField(r, level + (+$("al-slice0").value + +$("al-slice1").value) / 2);
  if (!f) throw new Error("IFC 모델에서 벽 높이 단면을 만들지 못했습니다");
  return f;
}
function applyAutoResult(res, s, r, floor, level, moveScan) {
  const M = poseMatrix(res);
  if (moveScan) moveScanWith(s, M, r);
  else {
    applyWorld(r.group, M.clone().invert());
    r.group.quaternion.setFromAxisAngle(UP, yawOf(r.group.quaternion));
  }
  seatFloor(s, r, moveScan);
  s.aligned = true;
  linkRef(s, r);
}
function showAutoResult(res, title) {
  const box = $("al-auto-result");
  box.classList.remove("hidden");
  const pct = Math.round(res.inlier * 100);
  const bar = res.inlier >= 0.6 ? "bg-emerald-500" : res.inlier >= 0.35 ? "bg-amber-500" : "bg-red-500";
  const t = Math.hypot(res.pose.tx, res.pose.tz);
  // only the wall match rate; the rest stays in the tooltip
  box.title = `${title} · 평균 편차 ${res.median != null ? (res.median * 1000).toFixed(0) + " mm" : "-"} · 회전 ${(res.pose.phi / DEG).toFixed(2)}° · 이동 ${(t * 1000).toFixed(0)} mm${res.pose.s !== 1 ? ` · 배율 ×${res.pose.s.toFixed(4)}` : ""}`;
  box.innerHTML = `<div class="flex items-center justify-between text-[10px] font-bold text-gray-600 mb-1"><span><i class="fa-solid fa-border-all mr-1"></i>벽 일치율</span><span>${pct}%</span></div>
    <div class="h-2 rounded-full bg-gray-200 overflow-hidden"><div class="h-full ${bar} rounded-full" style="width:${pct}%"></div></div>`;
}
function autoPrecheck() {
  const s = state.activeScan, r = state.activeRef;
  if (!s || !r) { toast("스캔과 도면/BIM이 모두 필요합니다"); return null; }
  if (!autoSupported(r)) { toast("입면/단면 도면은 자동 정합을 지원하지 않습니다 — 평면도나 IFC를 선택하세요"); return null; }
  if (blockedByLock(s, r)) return null;
  return { s, r, moveScan: moveTarget(s, r) === "scan" };
}
async function runRefine() {
  const ctx = autoPrecheck();
  if (!ctx) return;
  const { s, r, moveScan } = ctx;
  loader(true, "벽 단면으로 미세 보정 중...");
  await nextFrame();
  try {
    const level = refLevel(r);
    const { floor, S } = scanSlice(s);
    if (S.length < 400) throw new Error("벽 높이 단면에 점이 너무 적습니다 — 바닥 인식과 단면 높이를 확인하세요");
    const res = refine(refField(r, level), S, { phi: 0, tx: 0, tz: 0, s: 1 }, { withScale: $("al-scale").value === "1" && !s.metric });
    applyAutoResult(res, s, r, floor, level, moveScan);
    showAutoResult(res, "벽 단면 미세 보정 완료");
    updateXform();
  } catch (e) {
    console.error(e);
    toast(`미세 보정 실패: ${e.message || e}`, 5000);
  } finally {
    loader(false);
  }
}
let autoRun = null; // {results, idx, base matrices, ...} for cycling candidates
async function runAutoAlign() {
  const ctx = autoPrecheck();
  if (!ctx) return;
  const { s, r, moveScan } = ctx;
  if (r.type === "drawing" && !r.calibrated) toast("도면 축척이 보정되지 않았습니다 — 2단계에서 축척을 먼저 맞추면 결과가 정확해집니다", 5000);
  loader(true, "자동 정합: 벽 단면 추출 중...");
  await nextFrame();
  try {
    const level = refLevel(r);
    const { floor, S } = scanSlice(s);
    if (S.length < 400) throw new Error("벽 높이 단면에 점이 너무 적습니다 — 바닥 인식과 단면 높이를 확인하세요");
    const field = refField(r, level);
    const results = await globalSearch(field, S, { onProgress: (k) => loader(true, `자동 정합: 회전·위치 탐색 중... ${Math.round(k * 100)}%`) });
    s.group.updateMatrix(); r.group.updateMatrix();
    const others = linkedRefs(s).filter((o) => o !== r).map((o) => (o.group.updateMatrix(), [o, o.group.matrix.clone()]));
    autoRun = { s, r, results, idx: 0, floor, level, moveScan, others, scanBase: s.group.matrix.clone(), refBase: r.group.matrix.clone() };
    applyAutoCandidate(0);
  } catch (e) {
    console.error(e);
    toast(`자동 정합 실패: ${e.message || e}`, 5000);
  } finally {
    loader(false);
  }
}
function applyAutoCandidate(i) {
  const a = autoRun;
  a.idx = i;
  restoreMatrix(a.s.group, a.scanBase);
  restoreMatrix(a.r.group, a.refBase);
  for (const [o, m] of a.others) restoreMatrix(o.group, m);
  applyAutoResult(a.results[i], a.s, a.r, a.floor, a.level, a.moveScan);
  showAutoResult(a.results[i], `자동 정합 완료 — 후보 ${i + 1}/${a.results.length}`);
  $("al-auto-next").classList.toggle("hidden", a.results.length < 2);
  updateXform();
  fitView();
}
// ---- colour markers (stand-in for printed QR targets)
const MARKER_SENS = { strict: [12, 0.6], normal: [18, 0.48], loose: [28, 0.35] }; // hue tolerance (deg), min saturation
function markerHue() {
  const v = $("mk-color").value;
  if (v !== "custom") return +v;
  const c = new THREE.Color($("mk-custom").value);
  const hsl = {};
  c.getHSL(hsl, THREE.SRGBColorSpace);
  return hsl.h * 360;
}
$("mk-color").addEventListener("change", () => { $("mk-custom").classList.toggle("hidden", $("mk-color").value !== "custom"); renderAlign(); });
$("mk-sens").addEventListener("change", () => renderAlign());
$("mk-custom").addEventListener("change", () => renderAlign());
function markerWorld(s, m) { s.group.updateMatrixWorld(true); return m.local.clone().applyMatrix4(s.mesh.matrixWorld); }
function showMarkerPins(s) {
  if (s.markerPins) { dropLabels(s.markerPins); s.mesh.remove(s.markerPins); }
  s.markerPins = new THREE.Group();
  s.markers.forEach((m, i) => {
    const o = new THREE.Object3D();
    o.position.copy(m.local);
    o.add(label(`M${i + 1}`, "mk3d"));
    s.markerPins.add(o);
  });
  s.markerPins.visible = s.showMarkers !== false;
  s.mesh.add(s.markerPins);
}
function findMarkers(s) {
  const [tol, sat] = MARKER_SENS[$("mk-sens").value];
  const core = s.core?.box;
  const diag = core ? core.getSize(new THREE.Vector3()).length() : 10;
  const metric = s.metric || s.aligned;
  // the scan's own units: metres when metric, otherwise relative to the core size
  const scale = metric ? 1 / (s.group.scale.x || 1) : diag / 15;
  s.markers = detectMarkers(s.mesh, s.count, {
    hue: markerHue(), hueTol: tol, minSat: sat, minVal: 0.22,
    cell: 0.05 * scale, minCount: 12, maxSize: 2.0 * scale, box: core,
  });
  showMarkerPins(s);
  return s.markers;
}
$("mk-detect").onclick = async () => {
  const s = state.activeScan; if (!s) return toast("스캔을 먼저 올려주세요");
  loader(true, "스캔에서 색 마커를 찾는 중...");
  await nextFrame();
  try {
    const ms = findMarkers(s);
    toast(ms.length ? `마커 ${ms.length}개를 찾았습니다 (화면의 M1, M2 …)` : "마커를 찾지 못했습니다 — 색이나 민감도를 바꿔 보세요", 4500);
  } finally {
    loader(false);
    renderAll();
  }
};
function markerTol(s) {
  return s.metric || s.aligned ? 0.35 : 0.04 * (s.core ? s.core.box.getSize(new THREE.Vector3()).length() * s.group.scale.x : 10);
}
// no result box: a short toast with the headline, the full text in the console for checking
function showMarkerResult(html) {
  const d = document.createElement("div");
  d.innerHTML = html;
  const lines = [...d.querySelectorAll("p")].map((p) => p.textContent.trim()).filter(Boolean);
  console.info("[marker]", lines.join("\n"));
  toast(lines.slice(0, 1).concat(lines.filter((l) => l.startsWith("평균 오차") || l.includes("만 대응"))).join(" · "), 5000);
}
$("mk-to-ref").onclick = () => markersToRef();
async function markersToRef({ fitScale = false } = {}) {
  const s = state.activeScan, r = state.activeRef;
  if (!s || !r) return toast("스캔과 도면/BIM이 모두 필요합니다");
  if (!s.markers?.length) findMarkers(s);
  if (s.markers.length < 2) return toast("스캔에서 마커를 2개 이상 찾아야 합니다");
  if (r.refPts.length < 2) return toast("도면에 마커 위치를 기준점으로 2개 이상 지정하세요 ('기준점 지정')");
  if (blockedByLock(s, r)) return;
  if (s.locked) fitScale = false; // a locked scan keeps its scale
  const planar = isPlanarRef(r);
  // fitScale: the scan's scale follows the two outermost markers (even for a metric scan); the scan is never bent
  const withScale = fitScale || ($("al-scale").value === "1" && !s.metric);
  const src = s.markers.slice(0, 20).map((m) => markerWorld(s, m));
  const dst = r.refPts.map((p) => refPointWorld(r, p));
  let res = matchMarkers(src, dst, solveYawSimilarity, { withScale, planar, tol: markerTol(s) });
  if (!res) return toast("마커와 기준점의 배치가 일치하는 조합을 찾지 못했습니다", 5000);
  let S0 = null, outer = null;
  if (fitScale) {
    const dist = (a, b) => (planar ? Math.hypot(a.x - b.x, a.z - b.z) : a.distanceTo(b));
    for (let i = 0; i < res.pairs.length; i++) for (let j = i + 1; j < res.pairs.length; j++) {
      const d = dist(dst[res.pairs[i][1]], dst[res.pairs[j][1]]);
      if (!outer || d > outer.d) outer = { i, j, d };
    }
    const [mi, ki] = res.pairs[outer.i], [mj, kj] = res.pairs[outer.j];
    const k0 = outer.d / Math.max(1e-9, dist(src[mi], src[mj]));
    const P = res.pairs.map(([m]) => src[m]), Q = res.pairs.map(([, k]) => dst[k]);
    const c = P.reduce((a, p) => a.add(p), new THREE.Vector3()).divideScalar(P.length);
    S0 = aboutPoint(c, new THREE.Matrix4().makeScale(k0, k0, k0));
    const rigid = solveYawSimilarity(P.map((p) => p.clone().applyMatrix4(S0)), Q, { withScale: false, planar });
    res = { ...rigid, M: rigid.M.clone().multiply(S0), s: k0, pairs: res.pairs, rigidM: rigid.M };
    outer.names = [r.refPts[ki].label, r.refPts[kj].label];
  }
  const moveScan = moveTarget(s, r) === "scan";
  if (moveScan) moveScanWith(s, res.M, r);
  else {
    // the reference only turns and shifts; any scale change belongs to the scan
    if (S0) moveScanWith(s, S0, r);
    applyWorld(r.group, (res.rigidM || res.M).clone().invert());
    r.group.quaternion.setFromAxisAngle(UP, yawOf(r.group.quaternion));
  }
  if (planar) seatFloor(s, r, moveScan);
  s.aligned = true;
  linkRef(s, r);
  showMarkerResult(`<p class="font-bold text-gray-900"><i class="fa-solid fa-circle-check text-red-600 mr-1"></i>마커 매칭 정렬 완료 — ${res.pairs.length}쌍</p>
    <p>${res.pairs.map(([m, k]) => `${esc(r.refPts[k].label)} ↔ M${m + 1}`).join(" · ")}</p>
    <p>평균 오차(RMS) <b>${(res.rms * 1000).toFixed(0)} mm</b> · 회전 ${(res.theta / DEG).toFixed(2)}°${withScale ? ` · 배율 ×${res.s.toFixed(4)}` : ""}</p>
    ${outer ? `<p class="text-gray-500"><i class="fa-solid fa-maximize mr-1"></i>배율 기준: 최외곽 ${esc(outer.names[0])} ↔ ${esc(outer.names[1])} (${outer.d.toFixed(3)} m)</p>` : ""}
    <p class="text-gray-500">${res.pairs.map(([, k], i) => `${esc(r.refPts[k].label)} ${(res.residuals[i] * 1000).toFixed(0)}mm`).join(" · ")}</p>
    ${res.pairs.length < r.refPts.length ? `<p class="text-red-500">기준 마커 ${r.refPts.length}개 중 ${res.pairs.length}개만 대응됐습니다.</p>` : ""}`);
  updateXform();
  fitView();
  // wall-slice refinement would pull the markers apart again
  if (!fitScale && $("al-autorefine").checked && autoSupported(r)) await runRefine();
}
// ---- 2D drawing markers: mark 2+ marker positions on the plan once; the scan's colour markers are matched to
// them in any order. With only two markers the pairing (and a 180° flip) is ambiguous, so every candidate
// is scored against the drawing's wall lines and the best one wins.
async function pickDrawingMarkers(r) {
  const res = await pickOnDrawing(r, {
    title: "도면에 마커 위치 지정", min: 2, max: Infinity, labels: [],
    sub: "현장 마커(빨간 점·소화전 등)가 있는 위치를 도면에서 2개 이상 클릭 후 확인. 서로 멀리 떨어진 마커일수록 정확합니다.",
  });
  if (!res) return false;
  r.refPts = res.pts.map((p, i) => ({ label: `마커 ${i + 1}`, px: p.x, py: p.y, marker: true }));
  rebuildPins(r);
  renderRefPts();
  renderAlign();
  return true;
}
function wallScore(field, S, M) {
  // share of scan wall-slice points that land on drawing lines after moving the scan by M
  const e = M.elements, step = Math.max(1, Math.floor(S.length / 2 / 1500));
  let n = 0, hit = 0;
  for (let i = 0; i < S.length; i += 2 * step) {
    const x = S[i], z = S[i + 1];
    const wx = e[0] * x + e[8] * z + e[12], wz = e[2] * x + e[10] * z + e[14];
    n++;
    if (field.dist(wx, wz) < 0.08) hit++;
  }
  return n ? hit / n : 0;
}
async function alignDrawingMarkers() {
  const s = state.activeScan, r = state.activeRef;
  if (!s || r?.type !== "drawing") return toast("스캔과 2D 도면이 모두 필요합니다");
  if (blockedByLock(s, r)) return;
  if (!(r.refPts.length >= 2 && r.refPts.every((p) => p.marker)) && !(await pickDrawingMarkers(r))) return;
  loader(true, "스캔에서 마커를 찾고 도면 마커에 맞추는 중...");
  await nextFrame();
  try {
    if (!s.markers?.length) { findMarkers(s); renderAll(); }
    if (s.markers.length < 2) throw new Error("스캔에서 마커를 2개 이상 찾지 못했습니다 — 마커 색·민감도를 바꿔 보세요");
    const withScale = $("mk-dwg-scale").checked || !(r.calibrated && s.metric);
    const src = s.markers.slice(0, 12).map((m) => markerWorld(s, m));
    const dst = r.refPts.map((p) => refPointWorld(r, p));
    const flat = (v) => new THREE.Vector3(v.x, 0, v.z);
    const dist = (a, b) => Math.hypot(a.x - b.x, a.z - b.z);
    let cands = [];
    if (dst.length >= 3) {
      const res = matchMarkers(src, dst, solveYawSimilarity, { withScale, planar: true, tol: markerTol(s) });
      if (res) cands.push(res);
    }
    if (!cands.length) {
      // every ordered pair of scan markers onto the two outermost drawing markers
      let i0 = 0, j0 = 1, dmax = 0;
      for (let i = 0; i < dst.length; i++) for (let j = i + 1; j < dst.length; j++) if (dist(dst[i], dst[j]) > dmax) { dmax = dist(dst[i], dst[j]); i0 = i; j0 = j; }
      for (let a = 0; a < src.length; a++) for (let b = 0; b < src.length; b++) {
        if (a === b) continue;
        const ds = dist(src[a], src[b]);
        if (ds < 1e-6) continue;
        const k = dmax / ds;
        if (!withScale ? Math.abs(dmax - ds) > Math.max(markerTol(s), 0.05 * dmax) : k < 0.2 || k > 5) continue;
        const fit = solveYawSimilarity([flat(src[a]), flat(src[b])], [flat(dst[i0]), flat(dst[j0])], { withScale, planar: true });
        cands.push({ ...fit, pairs: [[a, i0], [b, j0]] });
      }
    }
    if (!cands.length) throw new Error("스캔 마커와 도면 마커의 간격이 맞는 조합이 없습니다 — 도면 축척이나 마커 위치를 확인하세요");
    // pick by the drawing's wall lines (a two-marker fit alone cannot tell A↔B from B↔A)
    let best = cands[0];
    if (cands.length > 1) {
      const field = drawingField(r);
      const { S } = scanSlice(s);
      for (const c of cands) c.walls = S.length ? wallScore(field, S, c.M) : 0;
      cands.sort((p, q) => q.walls - p.walls || p.rms - q.rms);
      best = cands[0];
    }
    moveScanWith(s, best.M, r); // the drawing stays put, the scan moves
    seatFloor(s, r, true);
    s.aligned = true;
    linkRef(s, r);
    updateXform();
    fitView();
    toast(`도면 마커 ${best.pairs.length}개로 정렬했습니다 — 회전 ${(best.theta / DEG).toFixed(1)}°${withScale ? ` · 배율 ×${best.s.toFixed(4)}` : ""}${best.walls != null ? ` · 벽 일치 ${Math.round(best.walls * 100)}%` : ""}`, 5000);
  } catch (e) {
    toast(e.message || String(e), 6000);
  } finally {
    loader(false);
  }
}
$("mk-dwg-run").onclick = () => alignDrawingMarkers();
$("mk-dwg-repick").onclick = async () => { const r = state.activeRef; if (r?.type === "drawing" && (await pickDrawingMarkers(r))) alignDrawingMarkers(); };

// IFC marker objects -> reference points, then the same order-free marker matching
$("mk-ifc-run").onclick = async () => {
  const s = state.activeScan, r = state.activeRef;
  if (!s || r?.type !== "bim") return toast("스캔과 IFC 모델이 모두 필요합니다");
  const g = r.markerOptions?.[+$("mk-ifc-group").value];
  if (!g) return toast("IFC에서 마커 객체를 찾지 못했습니다 — 마커 객체 이름에 'Marker' 또는 '마커'를 넣거나 목록에서 그룹을 고르세요", 6000);
  r.refPts = g.items.map((o, i) => ({ label: o.name || `${g.stem} ${i + 1}`, local: o.center.toArray() }));
  rebuildPins(r);
  renderRefPts();
  if (!s.markers?.length) {
    loader(true, "스캔에서 색 마커를 찾는 중...");
    await nextFrame();
    try { findMarkers(s); } finally { loader(false); }
    renderAll();
  }
  await markersToRef({ fitScale: $("mk-fit-scale").checked });
};

// scan markers -> new marker objects in the IFC (needs the scan aligned to this model first)
async function offerFile(filename, text) {
  const downloads = window.claude?.use ? await window.claude.use("downloads") : null;
  if (downloads) {
    // the artifact viewer only saves allow-listed types, so the .ifc travels inside a zip
    try {
      await downloads.save({ filename: filename.replace(/\.ifc$/i, ".zip"), data: zipOne(filename, new TextEncoder().encode(text)) });
      toast("저장했습니다 — zip을 풀어 나온 IFC를 다음부터 사용하세요", 5000);
    } catch (e) {
      if (e?.code !== "declined") toast(`저장하지 못했습니다 (${e?.code || e})`, 5000);
    }
    return;
  }
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([text], { type: "application/x-step" }));
  a.download = filename;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 10000);
  toast(`${filename} 를 저장했습니다`, 4000);
}
$("mk-export").onclick = async () => {
  const s = state.activeScan, r = state.activeRef;
  if (r?.type !== "bim") return toast("IFC 또는 STL 등 3D 모델을 기준으로 선택하세요");
  if (r.kind === "ifc" && (!r.ifcText || !r.coordMatrix)) return toast("이 IFC는 원본 정보가 없어 내보낼 수 없습니다 — IFC를 다시 올려 주세요", 5000);
  r.group.updateMatrixWorld(true);
  const toModel = r.group.matrixWorld.clone().invert();
  // sensor positions in the model's own space: scan markers (needs the scan aligned) or points picked on the model
  let markers;
  if ($("mk-src").value === "scan") {
    if (!s) return toast("스캔을 먼저 올려주세요");
    if (!s.markers?.length) return toast("먼저 '스캔에서 마커 찾기'로 마커를 찾으세요");
    if (!s.aligned) {
      const ok = await ask("스캔이 아직 모델에 정렬되지 않았습니다", "센서 위치는 지금 화면의 스캔 위치 그대로 기록됩니다. 기준점·자동 정합으로 스캔을 모델에 먼저 맞춘 뒤 내보내는 것을 권장합니다.\n\n그래도 내보낼까요?");
      if (!ok) return;
    }
    markers = s.markers.map((m, i) => ({ name: `Marker-${String(i + 1).padStart(2, "0")}`, tag: `M${i + 1}`, pos: markerWorld(s, m).applyMatrix4(toModel) }));
  } else {
    if (r.refPts.length < 1) return toast("'기준점 지정'으로 모델 위에 센서 위치를 먼저 찍으세요", 5000);
    markers = r.refPts.map((p, i) => ({ name: `Marker-${String(i + 1).padStart(2, "0")}`, tag: `M${i + 1}`, pos: new THREE.Vector3().fromArray(p.local) }));
  }
  loader(true, "IFC를 만드는 중...");
  await nextFrame();
  try {
    const base = r.name.replace(/\.[^.]+$/, "");
    let text, fileName;
    if (r.kind === "ifc") {
      text = addMarkersToIfc(r.ifcText, markers, { coordMatrix: r.coordMatrix });
      fileName = base + "_markers.ifc";
    } else {
      // STL / OBJ / ...: the model's own triangles become an IFC4 model, plus the sensors
      const meshes = r.group.children.filter((o) => o.isMesh).map((o, i) => ({
        name: `Model-${String(i + 1).padStart(2, "0")}`,
        positions: o.geometry.attributes.position.array,
        index: o.geometry.index?.array,
        color: o.material.userData.baseColor.toArray(),
      }));
      text = buildMeshIfc({ name: base + ".ifc", meshes, markers });
      fileName = base + "_sensors.ifc";
    }
    // show the new IFC exactly where this model is (IFC coordinates = this model's own space)
    loader(true, "만든 IFC를 불러오는 중...");
    const m = await loadIfc(new File([text], fileName));
    const r2 = addModel({ name: fileName, ...m, ifcText: text });
    restoreMatrix(r2.group, r.group.matrixWorld.clone().multiply(new THREE.Matrix4().fromArray(m.coordMatrix).invert()));
    styleRef(r2);
    r.visible = false; // the new IFC replaces the source on screen
    applyVisibility();
    if (s && s.linked?.has(r.id)) linkRef(s, r2);
    loader(false);
    await offerFile(fileName, text);
    showMarkerResult(`<p class="font-bold text-gray-900"><i class="fa-solid fa-file-export text-red-600 mr-1"></i>${r.kind === "ifc" ? "IFC에 센서" : "IFC로 변환 + 센서"} ${markers.length}개 — ${esc(fileName)}</p>
      <p>Marker-01 ~ Marker-${String(markers.length).padStart(2, "0")} (빨간 20 cm 정육면체, 이름으로 자동 인식). 화면에도 같은 자리에 띄워 두었습니다 — 이 IFC를 기준으로 'IFC 마커로 자동 정렬'을 바로 시험할 수 있습니다.</p>`);
    renderAll();
  } catch (e) {
    console.error(e);
    toast(`IFC 만들기 실패: ${e.message || e}`, 6000);
  } finally {
    loader(false);
  }
};

// ---- scan -> IFC (walls from the wall-height slice, floor slab, markers)
// ceiling: the strongest horizontal layer (5 cm bins) at least 1.8 m above the floor; a scanned ceiling
// makes a sharp peak there. Without one, the 98th percentile height. Also returns the scan's top.
function scanHeights(s, floorY) {
  s.group.updateMatrixWorld(true);
  let P = sampleSplats(s.mesh, s.count, 200_000);
  if (s.core) P = filterToBox(P, s.core.box, s.mesh.matrixWorld);
  const ys = [];
  for (let i = 1; i < P.length; i += 3) ys.push(P[i]);
  ys.sort((a, b) => a - b);
  const top = ys[Math.floor(ys.length * 0.995)] ?? floorY + 2.7;
  const lo = floorY + 1.8, bins = new Map();
  for (const y of ys) if (y >= lo && y <= top) { const k = Math.floor((y - lo) / 0.05); bins.set(k, (bins.get(k) || 0) + 1); }
  let best = null, sum = 0;
  for (const [k, c] of bins) { sum += c; if (!best || c > best.c) best = { k, c }; }
  const mean = bins.size ? sum / bins.size : 0;
  const ceil = best && best.c > 3 * mean ? lo + (best.k + 0.5) * 0.05 : ys[Math.floor(ys.length * 0.98)];
  return { ceil, top, ceilFromPeak: !!(best && best.c > 3 * mean) };
}
// floor slab: rectangle in the walls' main direction around every wall, or the scan's core box
function slabOutline(walls, s) {
  if (!walls.length) {
    const b = scanBox(s);
    return [[b.min.x, b.min.z], [b.max.x, b.min.z], [b.max.x, b.max.z], [b.min.x, b.max.z]];
  }
  let sx = 0, sy = 0;
  for (const w of walls) { const a = Math.atan2(w.b[1] - w.a[1], w.b[0] - w.a[0]) * 4, l = Math.hypot(w.b[0] - w.a[0], w.b[1] - w.a[1]); sx += Math.cos(a) * l; sy += Math.sin(a) * l; }
  const ang = Math.atan2(sy, sx) / 4, c = Math.cos(ang), sn = Math.sin(ang);
  let u0 = Infinity, u1 = -Infinity, v0 = Infinity, v1 = -Infinity, tmax = 0;
  for (const w of walls) for (const [x, z] of [w.a, w.b]) {
    const u = x * c + z * sn, v = -x * sn + z * c;
    u0 = Math.min(u0, u); u1 = Math.max(u1, u); v0 = Math.min(v0, v); v1 = Math.max(v1, v); tmax = Math.max(tmax, w.t);
  }
  u0 -= tmax / 2; v0 -= tmax / 2; u1 += tmax / 2; v1 += tmax / 2;
  return [[u0, v0], [u1, v0], [u1, v1], [u0, v1]].map(([u, v]) => [u * c - v * sn, u * sn + v * c]);
}
let generatedIfc = null;
const SB_DESC = {
  "voxel-height": "공간을 블록으로 나눠 칸마다 물체가 있는 높이만큼 블록을 세웁니다. 낮은 물체도 그 높이대로, 벽은 천장까지 나오고, 같은 높이의 이웃 칸은 큰 상자로 합칩니다.",
  "voxel-plan": "공간을 블록으로 나눠 바닥~천장 높이로 꽉 찬 칸만 벽 경계로 보고, 이웃 블록을 큰 상자로 합칩니다. 가구·조명은 빠지고 덩어리 수가 가장 적습니다.",
  "voxel-3d": "점이 모인 모든 블록(가구 포함)을 상자로 쌓습니다. 공간 전체의 덩어리를 대략 보고 싶을 때 씁니다.",
  faces: "벽면을 직선으로 검출해 두께·모서리까지 정리한 벽을 만듭니다. 가장 정확하지만 현장에 따라 벽이 많이 나올 수 있습니다.",
};
function renderSbMethod() {
  const m = $("sb-method").value;
  $("sb-method-desc").textContent = $("sb-method").title = SB_DESC[m];
  $("sb-size-wrap").classList.toggle("hidden", m === "faces");
  $("sb-face-opts").classList.toggle("hidden", m !== "faces");
}
$("sb-method").addEventListener("change", renderSbMethod);
renderSbMethod();
$("sb-run").onclick = async () => {
  const s = state.activeScan; if (!s) return toast("스캔을 먼저 올려주세요");
  if (!(s.metric || s.aligned)) {
    const ok = await ask("스캔 크기가 실측인지 알 수 없습니다", "Scaniverse(LiDAR) 스캔이거나 축척이 맞는 도면에 정렬된 스캔이어야 IFC 치수가 실제와 맞습니다.\n\n그래도 지금 크기 그대로 만들까요?");
    if (!ok) return;
  }
  loader(true, "스캔에서 바닥·벽을 추출하는 중...");
  await nextFrame();
  try {
    const floorY = scanFloorY(s);
    const { S } = scanSlice(s);
    if (S.length < 400) throw new Error("벽 높이 단면에 점이 너무 적습니다 — 바닥 인식과 단면 높이(3단계)를 확인하세요");
    const core = scanBox(s).getCenter(new THREE.Vector3());
    const method = $("sb-method").value;
    const conservative = $("sb-conservative").checked;
    // conservative: only long, densely scanned faces become walls
    const faces = extractFaces(S, method === "faces" && !conservative ? {} : { minLen: 1.0, density: 0.35 });
    const hs = scanHeights(s, floorY);
    let height = hs.ceil - floorY;
    if (!(height > 1.8 && height < 8)) height = 2.7;
    let walls, summary;
    if (method === "faces") {
      walls = facesToWalls(faces, [core.x, core.z], { thickness: +$("sb-thick").value || 0.2, orthogonal: $("sb-ortho").checked, conservative });
    } else {
      // blocks on a grid turned to the walls' main direction
      let sx = 0, sy = 0;
      for (const f of faces) { const a = Math.atan2(f.dz, f.dx) * 4, w = f.t1 - f.t0; sx += Math.cos(a) * w; sy += Math.sin(a) * w; }
      const angle = faces.length ? Math.atan2(sy, sx) / 4 : 0;
      loader(true, "블록(복셀)으로 경계를 만드는 중...");
      await nextFrame();
      s.group.updateMatrixWorld(true);
      let P = sampleSplats(s.mesh, s.count, 800_000);
      if (s.core) P = filterToBox(P, s.core.box, s.mesh.matrixWorld);
      const size = +$("sb-size").value;
      const mode = { "voxel-height": "height", "voxel-3d": "3d", "voxel-plan": "plan" }[method];
      // boxes take their own heights from the scan, up to the top of the scanned space
      const res = voxelBoxes(P, {
        floorY, ceilY: Math.max(hs.top, floorY + 1), angle, size, mode,
        ceilCut: hs.ceilFromPeak ? hs.ceil - 0.08 : Infinity, // a scanned ceiling is not an object
      });
      // tall thin boxes are walls; everything else is an object (furniture, fixtures, low items)
      walls = res.boxes.map((b, i) => {
        const wall = mode === "plan" || (mode === "height" && b.h >= 1.8 && b.t <= 0.6);
        return { ...b, label: `${wall ? "Wall" : "Object"}-${String(i + 1).padStart(3, "0")}`, kind: wall ? "wall" : "proxy", note: "voxel block" };
      });
      const tops = walls.map((w) => w.y0 + w.h - floorY);
      summary = `블록 ${size * 100} cm · 채워진 칸 ${res.cells.toLocaleString()}개 → 상자 ${walls.length}개로 합침 · 상자 높이 ${Math.min(...tops).toFixed(2)}~${Math.max(...tops).toFixed(2)} m`;
    }
    if (!walls.length) throw new Error("경계를 찾지 못했습니다 — 블록 크기를 키우거나 다른 생성 방식을 써 보세요");
    let markers = [];
    if ($("sb-markers").checked) {
      if (!s.markers) findMarkers(s);
      markers = s.markers.map((m, i) => ({ name: `Marker-${String(i + 1).padStart(2, "0")}`, tag: `M${i + 1}`, pos: markerWorld(s, m) }));
    }
    const name = s.name.replace(/\.[^.]+$/, "") + "_scan2bim.ifc";
    const text = buildScanIfc({ name, walls, floorY, height, slab: slabOutline(walls, s), markers });
    // show it right away as a BIM layer sitting exactly on the scan
    loader(true, "생성한 IFC를 불러오는 중...");
    const prev = state.refs.find((o) => o.generated && o.name === name);
    if (prev) removeRef(prev);
    const m = await loadIfc(new File([text], name));
    const r = addModel({ name, ...m, ifcText: text });
    r.generated = true;
    restoreMatrix(r.group, new THREE.Matrix4().fromArray(m.coordMatrix).invert()); // undo web-ifc's origin shift
    styleRef(r);
    linkRef(s, r); // built from the scan, so it is aligned to it already
    generatedIfc = { name, text };
    $("sb-save").classList.remove("hidden");
    const total = walls.reduce((a, w) => a + Math.hypot(w.b[0] - w.a[0], w.b[1] - w.a[1]), 0);
    const paired = walls.filter((w) => w.paired);
    const box = $("sb-result");
    box.classList.remove("hidden");
    const detail = summary
      ? `<p>${summary} · 바닥 슬래브 1 · 마커 ${markers.length}개</p>`
      : `<p>벽 ${walls.length}개 (총 ${total.toFixed(1)} m${paired.length ? ` · 양면 측정 ${paired.length}개, 평균 두께 ${(paired.reduce((a, w) => a + w.t, 0) / paired.length * 1000).toFixed(0)} mm` : ""}) · 벽 높이 ${height.toFixed(2)} m · 바닥 슬래브 1 · 마커 ${markers.length}개</p>` +
        (conservative ? `<p>정리: 벽면 후보 ${walls.stats.faces}개 → 한 줄로 합침 ${walls.stats.mergedFaces} · 비스듬한 면 제외 ${walls.stats.droppedOffGrid} · 중복 제거 ${walls.stats.duplicates} · 짧은 벽 제외 ${walls.stats.short}</p>` : "");
    box.innerHTML = `<p class="font-bold text-gray-900"><i class="fa-solid fa-circle-check text-blue-600 mr-1"></i>IFC 생성 완료 — ${esc(name)}</p>
      ${detail}
      <p>높이 기준: 바닥 ${floorY.toFixed(2)} m · 천장 ${(floorY + height).toFixed(2)} m (${hs.ceilFromPeak ? "천장면 인식" : "천장면 없음 — 상단 점 기준"}) · 스캔 최상단 ${hs.top.toFixed(2)} m</p>
      <p class="text-gray-500">화면에 BIM 레이어로 겹쳐 두었습니다. 결과가 마음에 들지 않으면 생성 방식·블록 크기·단면 높이(3단계)를 바꿔 다시 생성해 보세요.</p>`;
    renderAll();
  } catch (e) {
    console.error(e);
    toast(`IFC 생성 실패: ${e.message || e}`, 6000);
  } finally {
    loader(false);
  }
};
$("sb-save").onclick = () => generatedIfc && offerFile(generatedIfc.name, generatedIfc.text);

$("mk-to-scan").onclick = async () => {
  const s = state.activeScan, base = state.scans.find((o) => o.id === +$("mk-base").value);
  if (!s || !base || base === s) return toast("이전 공정 스캔을 선택하세요 (스캔이 2개 이상 필요)");
  loader(true, "두 스캔에서 마커를 찾는 중...");
  await nextFrame();
  try {
    if (!s.markers?.length) findMarkers(s);
    if (!base.markers?.length) findMarkers(base);
  } finally {
    loader(false);
    renderAll();
  }
  if (s.markers.length < 2 || base.markers.length < 2) return toast("두 스캔 모두에서 마커를 2개 이상 찾아야 합니다", 5000);
  const withScale = $("al-scale").value === "1" && !s.metric;
  const src = s.markers.slice(0, 20).map((m) => markerWorld(s, m));
  const dst = base.markers.slice(0, 20).map((m) => markerWorld(base, m));
  const res = matchMarkers(src, dst, solveYawSimilarity, { withScale, planar: false, tol: markerTol(base) });
  if (!res) return toast("두 스캔의 마커 배치가 일치하는 조합을 찾지 못했습니다", 5000);
  moveScanWith(s, res.M, null);
  s.aligned = s.aligned || base.aligned || base.metric;
  showMarkerResult(`<p class="font-bold text-gray-900"><i class="fa-solid fa-circle-check text-red-600 mr-1"></i>이전 스캔(${esc(base.name)})에 정렬 — 마커 ${res.pairs.length}쌍</p>
    <p>${res.pairs.map(([m, k]) => `M${m + 1} → 이전 M${k + 1}`).join(" · ")}</p>
    <p>평균 오차(RMS) <b>${(res.rms * 1000).toFixed(0)} mm</b> · 회전 ${(res.theta / DEG).toFixed(2)}°${withScale ? ` · 배율 ×${res.s.toFixed(4)}` : ""}</p>`);
  updateXform();
  fitView();
};
// snap a click to a detected marker when one is within `px` screen pixels
function markerNear(ev, px) {
  const rc = canvas.getBoundingClientRect();
  let best = null, bd = px;
  for (const s of state.scans) {
    if (!s.group.visible || !s.markers) continue;
    for (const m of s.markers) {
      const w = markerWorld(s, m), q = w.clone().project(camera);
      if (q.z > 1) continue;
      const d = Math.hypot(rc.left + (q.x + 1) / 2 * rc.width - ev.clientX, rc.top + (1 - q.y) / 2 * rc.height - ev.clientY);
      if (d < bd) { bd = d; best = w; }
    }
  }
  return best;
}

$("al-bimlevel").addEventListener("change", (e) => {
  const r = state.activeRef; if (r?.type !== "bim") return;
  r.levelIdx = +e.target.value;
  if (state.activeScan) { seatFloor(state.activeScan, r, moveTarget(state.activeScan, r) === "scan"); updateXform(); }
  toast(`BIM 기준 바닥 레벨을 바꾸고 스캔 바닥을 맞췄습니다`);
});
$("al-seat").addEventListener("click", () => {
  const s = state.activeScan, r = state.activeRef;
  if (!s || !r) return toast("스캔과 도면/BIM이 모두 필요합니다");
  if (blockedByLock(s, r)) return;
  seatFloor(s, r, moveTarget(s, r) === "scan");
  updateXform();
  toast(`스캔 바닥을 ${r.type === "bim" ? "BIM 바닥 레벨" : "도면 높이"}에 맞췄습니다 (높이 ${refLevel(r).toFixed(2)} m)`);
});

$("anchor-reset").addEventListener("click", () => {
  const s = state.activeScan; if (!s) return;
  s.anchor = null;
  s.linked = new Set();
  renderAlign();
  toast("앵커를 초기화했습니다 — 다음에 정렬하는 기준이 새 앵커가 됩니다");
});

$("al-auto").onclick = runAutoAlign;
$("al-refine").onclick = runRefine;
$("al-auto-next").onclick = () => autoRun && applyAutoCandidate((autoRun.idx + 1) % autoRun.results.length);

// ---- step 4: nudging & numeric transform
function nudgeTarget() {
  return $("nudge-target").value === "scan" ? state.activeScan?.group : state.activeRef?.group;
}
function nudge(kind, mult = 1) {
  const target = nudgeTarget();
  if (!target) return;
  const step = +$("nudge-step").value * mult;
  const rot = 0.25 * DEG * mult;
  const fwd = new THREE.Vector3();
  camera.getWorldDirection(fwd);
  fwd.y = 0;
  if (fwd.lengthSq() < 1e-6) fwd.set(0, 0, -1);
  fwd.normalize();
  const right = new THREE.Vector3().crossVectors(fwd, UP).normalize();
  const center = $("nudge-target").value === "scan" ? scanCenter(state.activeScan) : target.getWorldPosition(new THREE.Vector3());
  const T = (v) => new THREE.Matrix4().makeTranslation(v.x, v.y, v.z);
  let M;
  switch (kind) {
    case "x-": M = T(right.clone().multiplyScalar(-step)); break;
    case "x+": M = T(right.clone().multiplyScalar(step)); break;
    case "z-": M = T(fwd.clone().multiplyScalar(step)); break;
    case "z+": M = T(fwd.clone().multiplyScalar(-step)); break;
    case "y-": M = T(new THREE.Vector3(0, -step, 0)); break;
    case "y+": M = T(new THREE.Vector3(0, step, 0)); break;
    case "rot-": M = aboutPoint(center, new THREE.Matrix4().makeRotationY(rot)); break;
    case "rot+": M = aboutPoint(center, new THREE.Matrix4().makeRotationY(-rot)); break;
    case "s-": case "s+": { const k = 1 + (kind === "s+" ? 1 : -1) * 0.002 * mult; M = aboutPoint(center, new THREE.Matrix4().makeScale(k, k, k)); break; }
    default: return;
  }
  if (state.activeScan && target === state.activeScan.group) moveScanWith(state.activeScan, M, state.activeRef);
  else applyWorld(target, M);
  updateXform();
}
document.querySelectorAll("[data-nudge]").forEach((b) => b.addEventListener("click", (e) => nudge(b.dataset.nudge, e.shiftKey ? 10 : 1)));
window.addEventListener("keydown", (e) => {
  if (e.target.closest?.("input, textarea, select")) return;
  if (!$("modal-pick").classList.contains("hidden") || !$("modal-ask").classList.contains("hidden")) return;
  if (e.key === "Escape") return cancelPick();
  // no keyboard nudging: arrow keys used to move the scan / model by accident (use the step 3 buttons)
});

function updateXform() {
  const r = state.activeRef;
  $("xform-box").classList.toggle("opacity-40", !r);
  if (!r) return;
  const g = r.group, act = document.activeElement;
  const set = (id, v) => { if (act !== $(id)) $(id).value = v; };
  set("xf-x", g.position.x.toFixed(3));
  set("xf-z", g.position.z.toFixed(3));
  set("xf-y", g.position.y.toFixed(3));
  set("xf-rot", (yawOf(g.quaternion) / DEG).toFixed(2));
}
for (const id of ["xf-x", "xf-y", "xf-z", "xf-rot"]) {
  $(id).addEventListener("change", () => {
    const r = state.activeRef; if (!r) return;
    if (lockOwner(r.group)) { lockedToast(); return updateXform(); }
    r.group.position.set(+$("xf-x").value, +$("xf-y").value, +$("xf-z").value);
    r.group.quaternion.setFromAxisAngle(UP, +$("xf-rot").value * DEG);
  });
}

// =====================================================================
// boot
// =====================================================================
setView("plan", false);
renderAll();
// =====================================================================
// Ruler ("자"): click two points to measure. On IFC / model surfaces the cursor snaps to the object's
// outline: corners first, then the nearest point on an outline edge (feature edges > 25°).
// =====================================================================
const dim = { on: false, group: new THREE.Group(), a: null, list: [], snap: true, ortho: true };
scene.add(dim.group);
const mm = (v) => `${Math.round(v * 1000).toLocaleString()}`;
const snapMark = new THREE.Mesh(new THREE.SphereGeometry(1, 12, 8), new THREE.MeshBasicMaterial({ color: 0xf59e0b, depthTest: false }));
snapMark.renderOrder = 2004;
snapMark.visible = false;
snapMark.onBeforeRender = () => snapMark.scale.setScalar(Math.max(0.01, camera.position.distanceTo(snapMark.position) * 0.011));
// white ring around the snap point so the cursor reads on any background
const snapRing = new THREE.Mesh(new THREE.RingGeometry(1.35, 1.9, 32), new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false, transparent: true, side: THREE.DoubleSide }));
snapRing.renderOrder = 2003;
snapRing.onBeforeRender = () => snapRing.quaternion.copy(camera.quaternion);
snapMark.add(snapRing);
scene.add(snapMark);
// ruler mode dims the scene behind the ruler (a full-screen quad drawn just before the ruler pieces)
const dimLayer = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), new THREE.ShaderMaterial({
  vertexShader: "void main() { gl_Position = vec4(position.xy, 0.0, 1.0); }",
  fragmentShader: "void main() { gl_FragColor = vec4(0.0, 0.0, 0.0, 0.45); }",
  transparent: true, depthTest: false, depthWrite: false,
}));
dimLayer.renderOrder = 1999;
dimLayer.frustumCulled = false;
dimLayer.visible = false;
scene.add(dimLayer);
function setDimMode(on) {
  dim.on = on;
  cancelPending();
  dim.group.visible = on; // finished measurements come back when the ruler is reopened
  dimLayer.visible = on;
  canvas.style.cursor = on ? "crosshair" : "";
  $("tool-ruler").classList.toggle("bg-amber-50", on);
  $("tool-ruler").classList.toggle("ring-1", on);
  $("tool-ruler").classList.toggle("ring-amber-300", on);
  dim.a = null;
  snapMark.visible = false;
  elemLine.visible = false;
  showLive(null);
  if (on) {
    renderDimCard();
    toast("자: 두 점을 클릭해 거리를 잽니다 · Esc 나가기", 3000);
  } else $("dim-card").classList.add("hidden");
}
$("tool-ruler").onclick = () => setDimMode(!dim.on);
// narrow screens: the layer browser opens over the canvas
$("sidebar-toggle").onclick = () => $("sidebar").classList.toggle("open");

// outline edges of one IFC element (wall, slab, mass ...) in the model's own space, built once per element
// from that element's own triangles, so its outline stays complete where it touches other elements;
// meshes without element ids (STL / OBJ ...) use the whole mesh
function outlineEdges(r, mesh, id) {
  if (id == null) {
    mesh.userData.edges ??= new THREE.EdgesGeometry(mesh.geometry, 25).attributes.position.array;
    return mesh.userData.edges;
  }
  r.elemEdges ??= new Map();
  if (!r.elemEdges.has(id)) {
    const tri = [];
    for (const m of r.group.children) {
      const g = m.isMesh && m.geometry, ids = g?.userData?.ids;
      if (!ids) continue;
      const pos = g.attributes.position.array, idx = g.index?.array, nTri = (idx ? idx.length : pos.length / 3) / 3;
      for (let t = 0; t < nTri; t++) {
        if (ids[t] !== id) continue;
        for (let k = 0; k < 3; k++) { const v = idx ? idx[t * 3 + k] : t * 3 + k; tri.push(pos[v * 3], pos[v * 3 + 1], pos[v * 3 + 2]); }
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(tri, 3));
    r.elemEdges.set(id, new THREE.EdgesGeometry(g, 25).attributes.position.array);
    g.dispose();
  }
  return r.elemEdges.get(id);
}
// the outline of the element under the cursor, drawn so it is clear what the ruler snaps to
const elemLine = new LineSegments2(new LineSegmentsGeometry(), fatMaterial(0xffb020, 2.5, 0.95));
elemLine.renderOrder = 2002;
elemLine.frustumCulled = false;
elemLine.visible = false;
function showElemOutline(parent, edges, key) {
  if (!parent) { elemLine.visible = false; return; }
  if (elemLine.userData.key !== key) {
    elemLine.geometry.dispose();
    elemLine.geometry = new LineSegmentsGeometry().setPositions(edges);
    elemLine.userData.key = key;
  }
  if (elemLine.parent !== parent) parent.add(elemLine);
  elemLine.visible = true;
}
// nearest CAD endpoint / raster corner on a drawing sheet, in world space
function drawingSnap(r, hit, wpp) {
  r.group.updateMatrixWorld(true);
  const q = hit.point.clone().applyMatrix4(r.group.matrixWorld.clone().invert());
  const W = r.pxW / r.ppm, H = r.pxH / r.ppm;
  const px = (q.x + W / 2) * r.ppm, py = (r.orient === "plan" ? q.z + H / 2 : H / 2 - q.y) * r.ppm;
  const radius = (14 * wpp * r.ppm) / new THREE.Vector3().setFromMatrixScale(r.group.matrixWorld).x;
  const sp = r.snap ? snapEndpoint(r.snap, px, py, radius) : snapCorner(r.canvas, px, py, radius);
  if (!sp) return { p: hit.point.clone(), kind: "surface" };
  return { p: pixelToLocal(r, sp.x, sp.y).applyMatrix4(r.group.matrixWorld), kind: "vertex" };
}
// point under the cursor: BIM / drawing points snap to their outline (corners, then edges);
// scan points are taken as they are
function rulerPoint(ev) {
  rayFrom(ev);
  const rc = canvas.getBoundingClientRect();
  const wppAt = (p) => (2 * camera.position.distanceTo(p) * Math.tan((camera.fov * DEG) / 2)) / rc.height;
  const models = state.refs.filter((r) => r.type === "bim" && r.group.visible);
  const meshes = models.flatMap((r) => r.group.children.filter((o) => o.isMesh));
  const hit = raycaster.intersectObjects(meshes, false).find((h) => inCut(h.point));
  const sheets = state.refs.filter((r) => r.type === "drawing" && r.group.visible && r.mesh);
  const dHit = raycaster.intersectObjects(sheets.map((r) => r.mesh), false)[0];
  const sp = pickScan(ev);
  const dist = (p) => (p ? camera.position.distanceTo(p) : Infinity);
  // a drawing drawn "on top" wins wherever it is under the cursor
  const dr = dHit && sheets.find((r) => r.mesh === dHit.object);
  if (dr && (dr.opts.order === "top" || dist(dHit.point) <= Math.min(dist(hit?.point), dist(sp)) + 0.05)) {
    showElemOutline(null);
    const frame = () => yawOf(dr.group.quaternion);
    return { ...(dim.snap ? drawingSnap(dr, dHit, wppAt(dHit.point)) : { p: dHit.point.clone(), kind: "surface" }), frame };
  }
  const near = hit && dist(hit.point) <= dist(sp) + 0.05;
  // scan points are taken as they are (outline recognition on splats was not reliable enough to snap to)
  if (!near) { showElemOutline(null); return sp ? { p: sp, kind: "scan" } : null; }
  if (!dim.snap) { showElemOutline(null); return { p: hit.point.clone(), kind: "surface" }; }
  const m = hit.object;
  m.updateMatrixWorld(true);
  const model = models.find((o) => o.group === m.parent);
  const id = m.geometry.userData.ids ? m.geometry.userData.ids[hit.faceIndex] : null;
  const E = outlineEdges(model, m, id);
  showElemOutline(model.group, E, `${model.id}:${id ?? m.uuid}`);
  return snapEdges(E, m, hit.point, wppAt(hit.point));
}
// corners within ~10 px win, then the nearest point on an edge (14 px), else the surface point;
// E is a segment list in obj's own space
function snapEdges(E, obj, point, wpp) {
  obj.updateMatrixWorld(true);
  const mw = obj.matrixWorld, q = point.clone().applyMatrix4(mw.clone().invert());
  const R = (14 * wpp) / new THREE.Vector3().setFromMatrixScale(mw).x;
  let best = null, bestD = R, vert = null, vertD = R;
  for (let i = 0; i < E.length; i += 6) {
    const ax = E[i], ay = E[i + 1], az = E[i + 2], bx = E[i + 3], by = E[i + 4], bz = E[i + 5];
    // cheap reject: the cursor is far from this segment's box
    if (q.x < Math.min(ax, bx) - R || q.x > Math.max(ax, bx) + R || q.y < Math.min(ay, by) - R || q.y > Math.max(ay, by) + R || q.z < Math.min(az, bz) - R || q.z > Math.max(az, bz) + R) continue;
    const da = Math.hypot(q.x - ax, q.y - ay, q.z - az), db = Math.hypot(q.x - bx, q.y - by, q.z - bz);
    if (da < vertD) { vertD = da; vert = [ax, ay, az]; }
    if (db < vertD) { vertD = db; vert = [bx, by, bz]; }
    const dx = bx - ax, dy = by - ay, dz = bz - az, L2 = dx * dx + dy * dy + dz * dz;
    const t = L2 ? Math.max(0, Math.min(1, ((q.x - ax) * dx + (q.y - ay) * dy + (q.z - az) * dz) / L2)) : 0;
    const px = ax + dx * t, py = ay + dy * t, pz = az + dz * t, d = Math.hypot(q.x - px, q.y - py, q.z - pz);
    if (d < bestD) { bestD = d; best = [px, py, pz]; }
  }
  const frame = () => edgeFrame(E, mw);
  if (vert && vertD < R * 0.75) return { p: new THREE.Vector3(...vert).applyMatrix4(mw), kind: "vertex", frame };
  if (best) return { p: new THREE.Vector3(...best).applyMatrix4(mw), kind: "edge", frame };
  return { p: point.clone(), kind: "surface", frame };
}
const SNAP_COLOR = { vertex: 0xef4444, edge: 0xf59e0b, surface: 0xffffff, scan: 0x22c55e };
// axes of the object under the first point (SketchUp style): the dominant plan direction of its outline
// edges (lengths folded modulo 90°), returned as a yaw angle
function edgeFrame(E, mw) {
  const e = mw.elements;
  let c = 0, sn = 0;
  for (let i = 0; i < E.length; i += 6) {
    const lx = E[i + 3] - E[i], ly = E[i + 4] - E[i + 1], lz = E[i + 5] - E[i + 2];
    const dx = e[0] * lx + e[4] * ly + e[8] * lz, dy = e[1] * lx + e[5] * ly + e[9] * lz, dz = e[2] * lx + e[6] * ly + e[10] * lz;
    const h = Math.hypot(dx, dz);
    if (h < 1e-6 || Math.abs(dy) > 0.3 * h) continue; // plan-direction edges only
    const a = Math.atan2(dz, dx) * 4;
    c += h * Math.cos(a); sn += h * Math.sin(a);
  }
  return c || sn ? Math.atan2(sn, c) / 4 : null;
}
// fallback axes: the selected model / drawing, else the world
function defaultFrame() { return state.activeRef ? yawOf(state.activeRef.group.quaternion) : 0; }
const AXES = [
  { name: "x", color: 0xef4444, css: "#ef4444" },
  { name: "y", color: 0x3b82f6, css: "#3b82f6" },
  { name: "z", color: 0x22c55e, css: "#16a34a" },
];
function axisDirs(theta) {
  // x / z follow the object's plan directions, y is plumb (floor normal)
  return [new THREE.Vector3(Math.cos(theta), 0, Math.sin(theta)), new THREE.Vector3(0, 1, 0), new THREE.Vector3(-Math.sin(theta), 0, Math.cos(theta))];
}
// ortho (like CAD F8 / SketchUp inference), only while the first point is down: the second point slides
// along the snapped object's red / green (plan) or blue (plumb) axis, whichever the mouse direction follows
function toScreen(v) {
  const rc = canvas.getBoundingClientRect(), q = v.clone().project(camera);
  return new THREE.Vector2((q.x + 1) / 2 * rc.width + rc.left, (1 - q.y) / 2 * rc.height + rc.top);
}
function rulerTarget(ev) {
  const r = rulerPoint(ev);
  if (!dim.a || !dim.ortho) return r && { ...r, mode: null };
  const A = dim.a, sA = toScreen(A);
  const m = new THREE.Vector2(ev.clientX, ev.clientY).sub(sA);
  const dirs = axisDirs(dim.frame ?? 0);
  // the axis whose on-screen direction best matches the mouse
  let k = 1, bestDot = -1;
  if (m.length() > 4) {
    const mh = m.clone().normalize();
    dirs.forEach((d, i) => {
      const sd = toScreen(A.clone().add(d)).sub(sA);
      if (sd.length() < 1e-3) return;
      const dot = Math.abs(mh.dot(sd.normalize()));
      if (dot > bestDot) { bestDot = dot; k = i; }
    });
  }
  const d = dirs[k];
  let p;
  if (r && (r.kind === "vertex" || r.kind === "edge")) p = A.clone().addScaledVector(d, r.p.clone().sub(A).dot(d)); // snapped point, projected on the axis
  else {
    rayFrom(ev);
    p = new THREE.Vector3();
    raycaster.ray.distanceSqToSegment(A.clone().addScaledVector(d, -1000), A.clone().addScaledVector(d, 1000), null, p);
  }
  return { p, kind: r?.kind ?? "surface", mode: AXES[k].name, axis: AXES[k] };
}
const liveLabel = new THREE.Object3D();
liveLabel.add(label("", "dim3d"));
liveLabel.visible = false;
scene.add(liveLabel);
function showLive(a, b, mode) {
  liveLabel.visible = !!(a && b);
  if (!liveLabel.visible) { liveLabel.children[0].element.style.display = "none"; return; }
  liveLabel.position.copy(a).add(b).multiplyScalar(0.5);
  const el = liveLabel.children[0].element;
  el.style.display = "";
  const ax = AXES.find((o) => o.name === mode);
  el.textContent = `${ax ? ax.name.toUpperCase() + " " : ""}${mm(a.distanceTo(b))}`;
  el.style.background = ax ? ax.css : "";
}
let hoverQueued = null;
canvas.addEventListener("pointermove", (e) => {
  if (!dim.on || pick || e.buttons) return;
  if (hoverQueued) { hoverQueued = e; return; }
  hoverQueued = e;
  requestAnimationFrame(() => {
    const ev = hoverQueued; hoverQueued = null;
    const r = rulerTarget(ev);
    snapMark.visible = !!r;
    if (!r) { elemLine.visible = false; showLive(null); return; }
    snapMark.position.copy(r.p);
    snapMark.material.color.setHex(SNAP_COLOR[r.kind]);
    if (dim.a) {
      dim.rubber.geometry.setPositions([dim.a.x, dim.a.y, dim.a.z, r.p.x, r.p.y, r.p.z]);
      dim.rubber.material.color.setHex(r.axis ? r.axis.color : 0xffd166);
      showLive(dim.a, r.p, r.mode);
    }
  });
});
// drop a measurement that was started but not finished (Esc, close, toggle, clear)
function cancelPending() {
  if (dim.aDot) { dim.group.remove(dim.aDot); dim.aDot = null; }
  if (dim.rubber) { dim.group.remove(dim.rubber); dim.rubber.geometry.dispose(); fatMaterials.delete(dim.rubber.material); dim.rubber.material.dispose(); dim.rubber = null; }
  dim.a = null;
  showLive(null);
}
function rulerClick(ev) {
  const r = rulerTarget(ev);
  if (!r) return toast("스캔이나 모델 위를 클릭하세요", 2000);
  if (!dim.a) {
    dim.a = r.p.clone();
    dim.frame = (typeof r.frame === "function" ? r.frame() : null) ?? defaultFrame();
    const d = dot(0xffd166); d.position.copy(dim.a); d.renderOrder = 2003;
    dim.aDot = d;
    dim.rubber = fatLine(dim.a, dim.a, 0xffd166, 3);
    dim.group.add(d, dim.rubber);
    return;
  }
  const a = dim.a, b = r.p.clone(), aDot = dim.aDot;
  dim.aDot = null; // now part of the finished measurement
  cancelPending();
  const g = new THREE.Group();
  const line = fatLine(a, b, 0xffb020, 3.5);
  g.add(line, aDot);
  const d2 = dot(0xffd166); d2.position.copy(b); d2.renderOrder = 2003; g.add(d2);
  const m = { d: a.distanceTo(b), h: Math.hypot(a.x - b.x, a.z - b.z), v: Math.abs(a.y - b.y) };
  const o = new THREE.Object3D();
  o.position.copy(a).add(b).multiplyScalar(0.5);
  o.add(label(`${mm(m.d)}`, "dim3d"));
  g.add(o);
  dim.group.add(g);
  dim.list.push({ ...m, obj: g });
  renderDimCard();
}
function clearDim() {
  cancelPending();
  dropLabels(dim.group);
  dim.group.clear();
  dim.list = [];
}
function renderDimCard() {
  const card = $("dim-card");
  card.classList.remove("hidden");
  card.innerHTML = `<div class="flex items-center gap-2"><i class="fa-solid fa-ruler-combined text-amber-500"></i><span class="font-extrabold text-gray-900 text-xs flex-1">자</span>
      <label class="flex items-center gap-1 text-[10px] text-gray-500" title="도면·BIM 객체(벽체·매스) 외곽선의 꼭짓점·모서리에 스냅 (F3)"><input id="dim-snap" type="checkbox" ${dim.snap ? "checked" : ""} class="accent-[#f59e0b]"><i class="fa-solid fa-magnet"></i>스냅 F3</label>
      <label class="flex items-center gap-1 text-[10px] text-gray-500" title="첫 점을 찍은 뒤 스냅한 객체의 X(빨강)·Z(초록) 방향이나 수직 Y(파랑)으로만 (F8)"><input id="dim-ortho" type="checkbox" ${dim.ortho ? "checked" : ""} class="accent-[#f59e0b]">직교 F8</label>
      <button id="dim-clear" class="text-gray-300 hover:text-red-500 px-1" title="측정 모두 지우기"><i class="fa-solid fa-trash-can"></i></button>
      <button id="dim-close" class="text-gray-300 hover:text-gray-600 px-1" title="닫기"><i class="fa-solid fa-xmark"></i></button></div>
    ${dim.list.length ? `<div class="mt-2 space-y-0.5">${dim.list.map((m, i) => `<div class="flex items-baseline gap-1.5"><span class="text-[10px] text-gray-400 w-3">${i + 1}</span><b class="font-mono text-gray-900">${mm(m.d)} mm</b><span class="text-[10px] text-gray-400 truncate">수평 ${mm(m.h)} · 높이 ${mm(m.v)}</span></div>`).join("")}</div>` : ""}
    <p class="mt-1.5 text-[10px] text-gray-400"><span style="color:#ef4444">●</span> 꼭짓점 <span style="color:#f59e0b">●</span> 모서리 <span class="ml-1 font-bold" style="color:#ef4444">X</span> <span class="font-bold" style="color:#3b82f6">Y</span> <span class="font-bold" style="color:#16a34a">Z</span></p>`;
  $("dim-close").onclick = () => setDimMode(false);
  $("dim-clear").onclick = () => { clearDim(); renderDimCard(); };
  $("dim-snap").onchange = (e) => { dim.snap = e.target.checked; };
  $("dim-ortho").onchange = (e) => { dim.ortho = e.target.checked; };
}
window.addEventListener("keydown", (e) => {
  if (!dim.on) return;
  if (e.key === "F3") { e.preventDefault(); dim.snap = !dim.snap; renderDimCard(); return toast(`스냅 ${dim.snap ? "켜짐" : "꺼짐"} (F3)`, 1200); }
  if (e.key === "F8") { e.preventDefault(); dim.ortho = !dim.ortho; renderDimCard(); return toast(`직교 ${dim.ortho ? "켜짐" : "꺼짐"} (F8)`, 1200); }
  if (e.key === "Escape") setDimMode(false); // leaves ruler mode (and drops a measurement in progress)
});
// L: ruler on / off
window.addEventListener("keydown", (e) => {
  if (e.code !== "KeyL" || e.ctrlKey || e.metaKey || e.altKey || typing(e) || pick) return;
  e.preventDefault();
  setDimMode(!dim.on);
});

$("btn-land").onclick = async () => {
  if (walk) { exitWalk(); return toast("바닥 착륙을 해제했습니다"); }
  if (!state.scans.length && !state.refs.length) return toast("먼저 스캔이나 도면/BIM을 올려주세요");
  const pickFloor = (e) => {
    const sp = pickScan(e);
    const m = state.refs.filter((r) => r.type === "bim" && r.group.visible).map((r) => pickModel(e, r)).filter(Boolean);
    const all = [sp, ...m].filter(Boolean).sort((a, b) => camera.position.distanceTo(a) - camera.position.distanceTo(b));
    if (all[0]) return all[0];
    // nothing hit: the floor plane of the active scan (or the drawing sheet)
    rayFrom(e);
    const y = state.activeScan?.floorLocal ? scanFloorY(state.activeScan) : 0;
    const hit = new THREE.Vector3();
    return raycaster.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 1, 0), -y), hit) ? hit : null;
  };
  const pts = await pick3D({ text: "착륙할 바닥 위치를 클릭하세요", labels: ["착륙"], min: 1, max: 1, pickFn: pickFloor });
  if (!pts?.[0]) return;
  const p = pts[0];
  // a click near the detected scan floor lands exactly on it
  let floorY = p.y;
  const s = state.activeScan;
  if (s) { const fy = scanFloorAt(s, p.x, p.z, p.y + 0.3); if (fy != null && Math.abs(fy - p.y) < 0.5) floorY = fy; }
  landAt(p, floorY);
  toast("바닥 착륙 — WASD 걷기 · Shift 달리기 · 드래그 둘러보기 · Space 해제", 6000);
};
// Space: drop onto the floor right below the camera and walk there; Space again lifts off (stays in place)
window.addEventListener("keydown", (e) => {
  if (e.code !== "Space" || typing(e) || pick) return;
  e.preventDefault();
  if (walk) {
    const dir = new THREE.Vector3();
    camera.getWorldDirection(dir);
    exitWalk(false);
    controls.target.copy(camera.position).addScaledVector(dir, 3);
    return toast("바닥에서 떨어졌습니다 (자유 이동)", 1800);
  }
  if (!state.scans.length && !state.refs.length) return;
  const p = camera.position;
  const y = floorAt(p.x, p.z, p.y) ?? 0;
  landAt(p.clone(), y);
  toast("바닥 착륙 — WASD 걷기 · Shift 달리기 · 드래그 둘러보기 · Space 해제", 4000);
});
for (const id of ["btn-reset-view", "btn-top-view", "btn-side-view"]) $(id).addEventListener("click", () => exitWalk(false), true);

// =====================================================================
// Scan colours: original / greyscale / overlap with the BIM (scan near a model surface in colour,
// everything else grey). Splat colours are rewritten in place; the originals are kept for "원본".
// =====================================================================
const scanColor = { mode: "orig", tol: 0.05, color: "#22c55e" };
function splatStore(s) { return s.mesh.packedSplats || s.mesh.extSplats || null; }
function keepOriginalColors(s, store) {
  if (s.origColors) return;
  const c8 = new Uint8Array(s.count * 3);
  store.forEachSplat((i, _p, _s, _q, _o, c) => {
    c8[i * 3] = Math.max(0, Math.min(255, Math.round(c.r * 255)));
    c8[i * 3 + 1] = Math.max(0, Math.min(255, Math.round(c.g * 255)));
    c8[i * 3 + 2] = Math.max(0, Math.min(255, Math.round(c.b * 255)));
  });
  s.origColors = c8;
  s.origSh = store.maxSh ?? 3;
}
function refreshSplats(s, store, sh) {
  store.setMaxSh?.(sh); // view-dependent colour (spherical harmonics) would tint grey splats
  store.needsUpdate = true;
  s.mesh.updateGenerator?.();
  s.mesh.updateVersion?.();
}
// voxel keys of every visible BIM surface, sampled on a grid of `cell`
function bimSurfaceVoxels(cell) {
  const K = 65536, M = 131072, keys = new Set();
  const key = (x, y, z) => ((Math.floor(x / cell) + K) * M + (Math.floor(y / cell) + K)) * M + (Math.floor(z / cell) + K);
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
  for (const r of state.refs.filter((o) => o.type === "bim" && o.visible)) {
    r.group.updateMatrixWorld(true);
    for (const m of r.group.children.filter((o) => o.isMesh && !o.userData.dimOverlay)) {
      const pos = m.geometry.attributes.position.array, idx = m.geometry.index?.array;
      const nTri = (idx ? idx.length : pos.length / 3) / 3, mw = m.matrixWorld;
      for (let t = 0; t < nTri; t++) {
        const v = (k) => (idx ? idx[t * 3 + k] : t * 3 + k);
        a.fromArray(pos, v(0) * 3).applyMatrix4(mw); b.fromArray(pos, v(1) * 3).applyMatrix4(mw); c.fromArray(pos, v(2) * 3).applyMatrix4(mw);
        const n = Math.min(400, Math.ceil(Math.max(a.distanceTo(b), b.distanceTo(c), c.distanceTo(a)) / (cell * 0.7)));
        for (let i = 0; i <= n; i++) for (let j = 0; j <= n - i; j++) {
          const u = i / Math.max(1, n), w = j / Math.max(1, n), q = 1 - u - w;
          keys.add(key(a.x * q + b.x * u + c.x * w, a.y * q + b.y * u + c.y * w, a.z * q + b.z * u + c.z * w));
        }
      }
    }
  }
  return { keys, key };
}
async function applyScanColors() {
  const mode = scanColor.mode;
  if (!state.scans.length) return;
  const needBim = mode === "overlap";
  if (needBim && !state.refs.some((o) => o.type === "bim" && o.visible)) {
    toast("겹침 강조에는 IFC/3D 모델이 필요합니다 — 도면/BIM에 모델을 올려주세요", 4000);
  }
  loader(true, mode === "overlap" ? "스캔과 BIM이 겹치는 부분을 찾는 중..." : "스캔 색상을 바꾸는 중...");
  await nextFrame();
  try {
    const vox = needBim ? bimSurfaceVoxels(scanColor.tol) : null;
    const hi = new THREE.Color(scanColor.color);
    const col = new THREE.Color(), p = new THREE.Vector3();
    let inside = 0, total = 0;
    for (const s of state.scans) {
      const store = splatStore(s);
      if (!store?.setSplat) continue;
      keepOriginalColors(s, store);
      const O = s.origColors;
      s.group.updateMatrixWorld(true);
      const e = s.mesh.matrixWorld.elements;
      store.forEachSplat((i, c, sc, q, o) => {
        const r = O[i * 3] / 255, g = O[i * 3 + 1] / 255, bl = O[i * 3 + 2] / 255;
        if (mode === "orig") col.setRGB(r, g, bl);
        else {
          const y = 0.299 * r + 0.587 * g + 0.114 * bl;
          col.setRGB(y, y, y);
          if (vox) {
            p.set(e[0] * c.x + e[4] * c.y + e[8] * c.z + e[12], e[1] * c.x + e[5] * c.y + e[9] * c.z + e[13], e[2] * c.x + e[6] * c.y + e[10] * c.z + e[14]);
            total++;
            if (vox.keys.has(vox.key(p.x, p.y, p.z))) {
              inside++;
              // keep some shading so the surface still reads: highlight scaled by brightness
              const k = 0.45 + 0.55 * y;
              col.setRGB(hi.r * k, hi.g * k, hi.b * k);
            }
          }
        }
        store.setSplat(i, c, sc, q, o, col);
      });
      refreshSplats(s, store, mode === "orig" ? s.origSh : 0);
    }
    const info = $("overlap-info");
    info.classList.toggle("hidden", !vox);
    if (vox) info.innerHTML = `<i class="fa-solid fa-clone mr-1" style="color:${scanColor.color}"></i>BIM과 겹치는 스캔 <b>${total ? ((inside / total) * 100).toFixed(1) : 0}%</b> (허용 ${Math.round(scanColor.tol * 100)} cm)`;
  } finally {
    loader(false);
  }
}
function renderScanColorMode() {
  for (const b of document.querySelectorAll("#scan-color-mode [data-mode]")) {
    const on = b.dataset.mode === scanColor.mode;
    b.classList.toggle("bg-white", on); b.classList.toggle("shadow-xs", on); b.classList.toggle("text-gray-800", on); b.classList.toggle("text-gray-500", !on);
  }
  $("overlap-opts").classList.toggle("hidden", scanColor.mode !== "overlap");
}
for (const b of document.querySelectorAll("#scan-color-mode [data-mode]")) {
  b.onclick = async () => { scanColor.mode = b.dataset.mode; renderScanColorMode(); await applyScanColors(); };
}
$("overlap-tol").onchange = () => { scanColor.tol = +$("overlap-tol").value; applyScanColors(); };
$("overlap-color").onchange = () => { scanColor.color = $("overlap-color").value; applyScanColors(); };
$("overlap-run").onclick = () => applyScanColors();

for (const b of document.querySelectorAll("#front-mode [data-front]")) {
  b.onclick = () => {
    frontScan = b.dataset.front === "scan";
    spark.renderOrder = frontScan ? 10 : -10;
    state.refs.forEach(applyFront);
    for (const o of document.querySelectorAll("#front-mode [data-front]")) {
      const on = o === b;
      o.classList.toggle("bg-white", on); o.classList.toggle("shadow-xs", on); o.classList.toggle("text-gray-800", on); o.classList.toggle("text-gray-500", !on);
    }
  };
}

// =====================================================================
// Keep added files in this browser and open them again next time (with their placement)
// =====================================================================
let lastUnitMm = null, persistOrder = 0;
function layerMeta(o) {
  o.group.updateMatrix();
  const m = { id: o.persistId, kind: o.persistKind, order: o.persistOrder, matrix: o.group.matrix.toArray(), visible: o.visible !== false, unitMm: o.unitMm ?? null };
  if (o.persistKind === "scan") Object.assign(m, { floorLocal: o.floorLocal?.toArray() ?? null, aligned: !!o.aligned, metric: !!o.metric });
  else Object.assign(m, { opts: { ...o.opts }, refPts: o.refPts, ...(o.type === "drawing" ? { ppm: o.ppm, calibrated: !!o.calibrated, orient: o.orient } : {}) });
  return m;
}
function keepLayer(o, kind, file, restore) {
  o.persistKind = kind;
  o.persistId = restore?.id ?? newPersistId();
  o.persistOrder = restore?.order ?? Date.now() + persistOrder++;
  o.unitMm = restore?.unitMm ?? (kind === "ref" ? lastUnitMm : null);
  lastUnitMm = null;
  if (!restore) setTimeout(() => persistFile(o.persistId, file, layerMeta(o)), 0); // after the layer is set up
}
function restoreScan(s, m) {
  if (m.matrix) new THREE.Matrix4().fromArray(m.matrix).decompose(s.group.position, s.group.quaternion, s.group.scale);
  s.group.updateMatrixWorld(true);
  if (m.floorLocal) s.floorLocal = new THREE.Vector3().fromArray(m.floorLocal);
  s.aligned = m.aligned;
  s.metric = m.metric || s.metric;
  s.visible = m.visible;
}
function restoreRef(r, m) {
  if (r.type === "drawing") {
    if (m.ppm) { r.ppm = m.ppm; r.calibrated = m.calibrated; }
    if (m.orient) r.orient = m.orient;
  }
  if (m.opts) Object.assign(r.opts, m.opts);
  if (m.refPts) r.refPts = m.refPts;
  if (m.matrix) new THREE.Matrix4().fromArray(m.matrix).decompose(r.group.position, r.group.quaternion, r.group.scale);
  r.group.updateMatrixWorld(true);
  r.visible = m.visible;
  styleRef(r);
  rebuildPins(r);
}
// placement changes all the time (alignment, nudges, colours); save it every few seconds and on leaving
let lastMetaJson = "";
function saveLayerMeta() {
  const list = [...state.scans, ...state.refs].filter((o) => o.persistId).map(layerMeta);
  const json = JSON.stringify(list);
  if (json === lastMetaJson) return;
  lastMetaJson = json;
  persistMeta(list);
}
setInterval(saveLayerMeta, 3000);
addEventListener("pagehide", saveLayerMeta);
document.addEventListener("visibilitychange", () => document.visibilityState === "hidden" && saveLayerMeta());
(async () => {
  const saved = await persistLoadAll();
  if (!saved.length) return;
  for (const { meta, file } of saved) {
    if (meta.kind === "scan") await onScanFiles([file], meta);
    else await onRefFile(file, meta);
  }
  applyVisibility();
  renderAll();
  fitView();
})();

window.__app = { state, scene, applyScanColors, scanColor, camera, controls, landAt, exitWalk, getWalk: () => walk, solveYawSimilarity, markerWorld, refPointWorld, pixelToLocal, pickScan, view };

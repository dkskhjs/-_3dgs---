// IFC -> three.js meshes via web-ifc (WebAssembly). web-ifc returns Y-up geometry in metres,
// which matches the viewer's site coordinates. Meshes are merged per colour to keep draw calls low.
import * as THREE from "three";
import { hsv } from "./markers.js";

const WEB_IFC = window.__LIBS?.ifc ?? "https://cdn.jsdelivr.net/npm/web-ifc@0.0.77/"; // window.__LIBS: self-hosted copies
let api, W;

// STEP string escapes: \X2\hhhh…\X0\ (UTF-16, e.g. Korean names), \X\hh (Latin-1), \S\c
function decodeIfcString(s) {
  if (typeof s !== "string") return "";
  return s
    .replace(/\\X2\\([0-9A-F]+)\\X0\\/gi, (_, hex) => String.fromCharCode(...hex.match(/.{4}/g).map((h) => parseInt(h, 16))))
    .replace(/\\X\\([0-9A-F]{2})/gi, (_, h) => String.fromCharCode(parseInt(h, 16)))
    .replace(/\\S\\(.)/g, (_, c) => String.fromCharCode(c.charCodeAt(0) + 128));
}

export async function loadIfc(file, onProgress) {
  if (!api) {
    W = await import(WEB_IFC + "web-ifc-api.js");
    api = new W.IfcAPI();
    api.SetWasmPath(WEB_IFC, true);
    await api.Init(undefined, true);
  }
  const modelID = api.OpenModel(new Uint8Array(await file.arrayBuffer()), { COORDINATE_TO_ORIGIN: true });
  const buckets = new Map(); // colour key -> {color, opacity, pos[], nrm[], idx[]}
  const m4 = new THREE.Matrix4(), n3 = new THREE.Matrix3(), v = new THREE.Vector3(), n = new THREE.Vector3();
  const elements = new Map(); // expressID -> {box, color}: per-object extents for marker objects
  let coordMatrix = null;
  const pieces = []; // small saturated geometry pieces: marker objects found by colour, whatever their names
  let pieceVerts = 0;
  try {
    api.StreamAllMeshes(modelID, (mesh, i, total) => {
      if (onProgress && i % 200 === 0) onProgress(i / total);
      let el = elements.get(mesh.expressID);
      if (!el) elements.set(mesh.expressID, (el = { box: new THREE.Box3(), color: null }));
      const geoms = mesh.geometries;
      for (let g = 0; g < geoms.size(); g++) {
        const pg = geoms.get(g);
        const geom = api.GetGeometry(modelID, pg.geometryExpressID);
        const verts = api.GetVertexArray(geom.GetVertexData(), geom.GetVertexDataSize());
        const index = api.GetIndexArray(geom.GetIndexData(), geom.GetIndexDataSize());
        const c = pg.color;
        el.color ??= [c.x, c.y, c.z];
        const key = `${c.x.toFixed(3)},${c.y.toFixed(3)},${c.z.toFixed(3)},${c.w.toFixed(2)}`;
        let b = buckets.get(key);
        if (!b) buckets.set(key, (b = { color: new THREE.Color(c.x, c.y, c.z), opacity: c.w, pos: [], nrm: [], idx: [], ids: [] }));
        m4.fromArray(pg.flatTransformation);
        n3.getNormalMatrix(m4);
        const base = b.pos.length / 3;
        const [, sat] = hsv(c.x, c.y, c.z);
        const piece = sat > 0.3 && pieceVerts < 3_000_000 ? { color: [c.x, c.y, c.z], verts: [], box: new THREE.Box3() } : null;
        for (let k = 0; k < verts.length; k += 6) {
          v.set(verts[k], verts[k + 1], verts[k + 2]).applyMatrix4(m4);
          el.box.expandByPoint(v);
          if (piece) { piece.verts.push(v.x, v.y, v.z); piece.box.expandByPoint(v); }
          n.set(verts[k + 3], verts[k + 4], verts[k + 5]).applyMatrix3(n3).normalize();
          b.pos.push(v.x, v.y, v.z);
          b.nrm.push(n.x, n.y, n.z);
        }
        for (let k = 0; k < index.length; k++) b.idx.push(base + index[k]);
        for (let k = 0; k < index.length; k += 3) b.ids.push(mesh.expressID); // triangle -> IFC element
        if (piece && !piece.box.isEmpty()) {
          const sz = piece.box.getSize(new THREE.Vector3());
          if (Math.max(sz.x, sz.y, sz.z) < 3) { piece.verts = Float32Array.from(piece.verts); pieceVerts += piece.verts.length / 3; pieces.push(piece); }
        }
        geom.delete?.();
      }
    });
    // web-ifc's model -> viewer transform (origin shift, Z-up -> Y-up), needed to write back into the IFC
    coordMatrix = api.GetCoordinationMatrix(modelID);
    // object identity: IFC class, Name / ObjectType / Description / Tag
    for (const [id, el] of elements) {
      try {
        el.type = api.GetNameFromTypeCode(api.GetLineType(modelID, id)) || "";
        const line = api.GetLine(modelID, id);
        el.name = decodeIfcString(line?.Name?.value);
        el.objectType = decodeIfcString(line?.ObjectType?.value);
        el.desc = decodeIfcString(line?.Description?.value);
        el.tag = decodeIfcString(line?.Tag?.value);
      } catch {
        el.type ??= ""; el.name ??= "";
      }
    }
    var { quantities, units } = readQuantities(modelID);
  } finally {
    api.CloseModel(modelID);
  }
  const group = new THREE.Group();
  const materials = [];
  for (const b of buckets.values()) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(b.pos, 3));
    geo.setAttribute("normal", new THREE.Float32BufferAttribute(b.nrm, 3));
    geo.setIndex(b.pos.length / 3 > 65535 ? new THREE.Uint32BufferAttribute(b.idx, 1) : new THREE.Uint16BufferAttribute(b.idx, 1));
    geo.computeBoundingSphere();
    geo.userData.ids = Uint32Array.from(b.ids);
    const mat = new THREE.MeshLambertMaterial({ color: b.color, transparent: true, side: THREE.DoubleSide });
    mat.userData.baseColor = b.color.clone();
    mat.userData.baseOpacity = b.opacity;
    materials.push(mat);
    group.add(new THREE.Mesh(geo, mat));
  }
  if (!group.children.length) throw new Error("IFC에서 형상을 찾지 못했습니다");
  const objects = [...elements].filter(([, el]) => !el.box.isEmpty()).map(([id, el]) => ({
    id, type: el.type, name: el.name, objectType: el.objectType || "", desc: el.desc || "", tag: el.tag || "", color: el.color,
    center: el.box.getCenter(new THREE.Vector3()), size: el.box.getSize(new THREE.Vector3()),
  }));
  return { group, materials, objects, pieces, coordMatrix, quantities, units, info: `IFC 요소 ${objects.length.toLocaleString()}개` };
}

/** opacity 0..1; ghost = single translucent colour so the scan stays readable */
// ---- design dimensions stored in the IFC: quantity sets (Qto_*, BaseQuantities) and dimension-like
// properties (Revit "Dimensions": Length / Width / Height / Area / Volume ...), converted to metres
const DIM_NAME = /length|width|height|depth|thickness|perimeter|area|volume|span|길이|폭|너비|높이|깊이|두께|둘레|면적|체적|부피/i;
const SI_PREFIX = { EXA: 1e18, PETA: 1e15, TERA: 1e12, GIGA: 1e9, MEGA: 1e6, KILO: 1e3, HECTO: 1e2, DECA: 10, DECI: 0.1, CENTI: 0.01, MILLI: 1e-3, MICRO: 1e-6, NANO: 1e-9 };
const CONV = { FOOT: 0.3048, INCH: 0.0254, YARD: 0.9144, "SQUARE FOOT": 0.09290304, "SQUARE INCH": 0.00064516, "CUBIC FOOT": 0.0283168466, "CUBIC INCH": 1.6387064e-5 };
function readUnits(modelID) {
  const u = { len: 1, area: 1, vol: 1 };
  const key = { LENGTHUNIT: ["len", 1], AREAUNIT: ["area", 2], VOLUMEUNIT: ["vol", 3] };
  const ids = (t) => { try { const v = api.GetLineIDsWithType(modelID, t); return Array.from({ length: v.size() }, (_, i) => v.get(i)); } catch { return []; } };
  for (const id of ids(W.IFCSIUNIT)) {
    const l = api.GetLine(modelID, id), k = key[l.UnitType?.value];
    if (k) u[k[0]] = (SI_PREFIX[l.Prefix?.value] ?? 1) ** k[1];
  }
  for (const id of ids(W.IFCCONVERSIONBASEDUNIT)) {
    const l = api.GetLine(modelID, id), k = key[l.UnitType?.value];
    const f = CONV[(decodeIfcString(l.Name?.value) || "").toUpperCase()];
    if (k && f) u[k[0]] = f;
  }
  return u;
}
function readQuantities(modelID) {
  const quantities = new Map(); // expressID -> [{set, name, kind: len|area|vol|count, value (m, m², m³)}]
  let units = { len: 1, area: 1, vol: 1 };
  try {
    units = readUnits(modelID);
    const kindOfQ = { IFCQUANTITYLENGTH: ["len", "LengthValue"], IFCQUANTITYAREA: ["area", "AreaValue"], IFCQUANTITYVOLUME: ["vol", "VolumeValue"], IFCQUANTITYCOUNT: ["count", "CountValue"] };
    const rels = api.GetLineIDsWithType(modelID, W.IFCRELDEFINESBYPROPERTIES);
    for (let i = 0; i < rels.size(); i++) {
      const rel = api.GetLine(modelID, rels.get(i));
      const defId = rel.RelatingPropertyDefinition?.value;
      if (!defId) continue;
      const defType = String(api.GetNameFromTypeCode(api.GetLineType(modelID, defId))).toUpperCase();
      const def = api.GetLine(modelID, defId);
      const set = decodeIfcString(def.Name?.value) || "";
      const list = [];
      if (defType === "IFCELEMENTQUANTITY") {
        for (const q of def.Quantities || []) {
          const ql = api.GetLine(modelID, q.value);
          const k = kindOfQ[String(api.GetNameFromTypeCode(api.GetLineType(modelID, q.value))).toUpperCase()];
          const v = k && ql[k[1]]?.value;
          if (typeof v === "number") list.push({ set, name: decodeIfcString(ql.Name?.value), kind: k[0], value: k[0] === "count" ? v : v * units[k[0]] });
        }
      } else if (defType === "IFCPROPERTYSET") {
        for (const p of def.HasProperties || []) {
          const pl = api.GetLine(modelID, p.value);
          const name = decodeIfcString(pl.Name?.value), nv = pl.NominalValue;
          if (!nv || typeof nv.value !== "number" || !DIM_NAME.test(name)) continue;
          const t = String(nv.name || nv.label || "").toUpperCase();
          const kind = /AREA|면적/.test(t + name.toUpperCase()) ? "area" : /VOLUME|체적|부피/.test(t + name.toUpperCase()) ? "vol" : "len";
          list.push({ set, name, kind, value: nv.value * units[kind] });
        }
      }
      if (!list.length) continue;
      for (const o of rel.RelatedObjects || []) {
        const id = o.value;
        if (!quantities.has(id)) quantities.set(id, []);
        quantities.get(id).push(...list);
      }
    }
  } catch (e) {
    console.warn("IFC quantities:", e);
  }
  return { quantities, units };
}

export function styleModel(model, { opacity, ghost, color }) {
  for (const m of model.materials) {
    // single colour, except strongly coloured parts (sensor / marker objects) which keep their own colour
    const keep = ghost && hsv(m.userData.baseColor.r, m.userData.baseColor.g, m.userData.baseColor.b)[1] > 0.6;
    m.color.copy(ghost && !keep ? new THREE.Color(color) : m.userData.baseColor);
    m.opacity = opacity * (ghost ? 1 : m.userData.baseOpacity);
    m.depthWrite = m.opacity >= 0.99;
    m.needsUpdate = true;
  }
}

/**
 * Candidate floor levels of a model: upward-facing horizontal area accumulated per 5 cm of height
 * (slab tops, floor finishes). Heights are in the model group's own space. Sorted bottom to top.
 */
export function floorLevels(model) {
  const bins = new Map();
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3(), n = new THREE.Vector3();
  for (const mesh of model.group.children) {
    if (!mesh.isMesh) continue;
    const pos = mesh.geometry.attributes.position, nrm = mesh.geometry.attributes.normal, idx = mesh.geometry.index;
    const tri = idx ? idx.count / 3 : pos.count / 3;
    for (let t = 0; t < tri; t++) {
      const i0 = idx ? idx.getX(t * 3) : t * 3, i1 = idx ? idx.getX(t * 3 + 1) : t * 3 + 1, i2 = idx ? idx.getX(t * 3 + 2) : t * 3 + 2;
      a.fromBufferAttribute(pos, i0); b.fromBufferAttribute(pos, i1); c.fromBufferAttribute(pos, i2);
      n.subVectors(b, a).cross(c.clone().sub(a));
      const len = n.length();
      if (len < 1e-9 || Math.abs(n.y) / len < 0.97) continue; // horizontal faces only
      // which side faces up comes from IFC's vertex normals: triangle winding is not consistent
      if (nrm && nrm.getY(i0) + nrm.getY(i1) + nrm.getY(i2) <= 0) continue;
      const key = Math.round((a.y + b.y + c.y) / 3 / 0.05);
      bins.set(key, (bins.get(key) || 0) + len / 2);
    }
  }
  // merge neighbouring bins (within 10 cm) into levels
  const keys = [...bins.keys()].sort((x, y) => x - y);
  const levels = [];
  for (const k of keys) {
    const last = levels.at(-1);
    if (last && k - last.k <= 2) { last.wy += k * 0.05 * bins.get(k); last.area += bins.get(k); last.k = k; }
    else levels.push({ k, wy: k * 0.05 * bins.get(k), area: bins.get(k) });
  }
  const out = levels.map((l) => ({ y: l.wy / l.area, area: l.area }));
  const max = Math.max(0, ...out.map((l) => l.area));
  return out.filter((l) => l.area >= max * 0.05 && l.area >= 1);
}

// ---------------------------------------------------------------- marker objects
const MARKER_WORDS = /marker|마커|target|타겟|qr|소화전|hydrant|fire.?hose|fire.?ext|표식|표지|기준점|control.?point|survey|측량/i;

/**
 * Groups small IFC objects that could be site markers: same IFC class + name stem ("Marker-01",
 * "Marker-02" -> "Marker"). Groups whose name, type or class mention markers come first.
 */
export function markerGroups(objects) {
  const groups = new Map();
  for (const o of objects) {
    if (Math.max(o.size.x, o.size.y, o.size.z) > 3) continue; // markers are small objects
    const label = o.name || o.objectType || "(이름 없음)";
    const stem = label.replace(/[\s_\-#.()]*\d+[)]?$/, "").trim() || label;
    const key = `${o.type}|${stem}`;
    let g = groups.get(key);
    if (!g) groups.set(key, (g = { key, type: o.type, stem, items: [], marker: false }));
    g.items.push(o);
    if (MARKER_WORDS.test(`${o.name} ${o.objectType} ${o.desc} ${o.tag}`) || o.type.toUpperCase() === "IFCFIRESUPPRESSIONTERMINAL") g.marker = true;
  }
  const out = [...groups.values()].filter((g) => g.items.length >= 2 && g.items.length <= 60);
  // objects whose names mark them as markers belong together even when each name is different
  const named = objects.filter((o) => Math.max(o.size.x, o.size.y, o.size.z) <= 3 && MARKER_WORDS.test(`${o.name} ${o.objectType} ${o.desc} ${o.tag}`));
  if (named.length >= 2 && !out.some((g) => g.marker && g.items.length === named.length)) {
    out.push({ key: "named", type: "이름에 마커 표시", stem: "마커 이름 객체", items: named, marker: true });
  }
  return out.sort((a, b) => (b.marker - a.marker) || (a.items.length - b.items.length));
}

/**
 * Colour-based IFC markers: geometry pieces near the target hue, split into spatially connected
 * clusters (several spheres inside one object still come out separately). Model-space centres.
 */
export function colorMarkers(pieces, { hue, hueTol, minSat }) {
  const pts = [];
  for (const p of pieces) {
    const [h, s, v] = hsv(...p.color);
    const dh = Math.min(Math.abs(h - hue), 360 - Math.abs(h - hue));
    if (dh > hueTol || s < minSat || v < 0.15) continue;
    for (let i = 0; i < p.verts.length; i += 3) pts.push(p.verts[i], p.verts[i + 1], p.verts[i + 2]);
  }
  if (!pts.length) return [];
  const cell = 0.12, grid = new Map(), key = (a, b, c) => `${a},${b},${c}`;
  for (let i = 0; i < pts.length; i += 3) {
    const k = key(Math.floor(pts[i] / cell), Math.floor(pts[i + 1] / cell), Math.floor(pts[i + 2] / cell));
    let arr = grid.get(k);
    if (!arr) grid.set(k, (arr = []));
    arr.push(i);
  }
  const seen = new Set(), out = [];
  for (const start of grid.keys()) {
    if (seen.has(start)) continue;
    const stack = [start], box = new THREE.Box3(), v = new THREE.Vector3();
    seen.add(start);
    let n = 0;
    while (stack.length) {
      const k = stack.pop();
      for (const i of grid.get(k)) { box.expandByPoint(v.set(pts[i], pts[i + 1], pts[i + 2])); n++; }
      const [a, b, c] = k.split(",").map(Number);
      for (let x = -1; x <= 1; x++) for (let y = -1; y <= 1; y++) for (let z = -1; z <= 1; z++) {
        const nk = key(a + x, b + y, c + z);
        if (grid.has(nk) && !seen.has(nk)) { seen.add(nk); stack.push(nk); }
      }
    }
    const size = box.getSize(new THREE.Vector3());
    // markers are small and compact; thin roof tiles, trims and long members are not
    const mx = Math.max(size.x, size.y, size.z), mn = Math.min(size.x, size.y, size.z);
    if (mx <= 1.5 && mn >= mx * 0.2) out.push({ center: box.getCenter(new THREE.Vector3()), size, count: n });
  }
  // stable order: by height, then x, then z
  out.sort((a, b) => a.center.y - b.center.y || a.center.x - b.center.x || a.center.z - b.center.z);
  return out.map((m, i) => ({ ...m, name: `색 마커 ${i + 1}` }));
}

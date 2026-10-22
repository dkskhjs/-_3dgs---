// Automatic plan alignment: a horizontal slice of the scan (wall height) is matched against the
// lines of a plan drawing (or a horizontal section through a BIM model) using a distance field.
//  - globalSearch: no initial guess — brute force over rotation x translation, then refine
//  - refine:       local pattern search from the current pose (fixes click errors)
// Poses are 2D corrections applied to the scan in world space about the slice centroid c:
//   p' = Ry(phi) * s * (p - c) + c + t
import * as THREE from "three";

// ---------------------------------------------------------------- distance field
// exact squared Euclidean distance transform (Felzenszwalb & Huttenlocher), in cell units
function edt1d(f, n, d, v, z) {
  let k = 0;
  v[0] = 0; z[0] = -Infinity; z[1] = Infinity;
  for (let q = 1; q < n; q++) {
    let s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
    while (s <= z[k]) { k--; s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]); }
    k++; v[k] = q; z[k] = s; z[k + 1] = Infinity;
  }
  k = 0;
  for (let q = 0; q < n; q++) { while (z[k + 1] < q) k++; d[q] = (q - v[k]) ** 2 + f[v[k]]; }
}
function distanceTransform(mask, w, h) {
  const INF = 1e12, n = Math.max(w, h);
  const out = new Float32Array(w * h);
  const f = new Float64Array(n), d = new Float64Array(n), v = new Int32Array(n), z = new Float64Array(n + 1);
  for (let x = 0; x < w; x++) {
    for (let y = 0; y < h; y++) f[y] = mask[y * w + x] ? 0 : INF;
    edt1d(f, h, d, v, z);
    for (let y = 0; y < h; y++) out[y * w + x] = d[y];
  }
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) f[x] = out[y * w + x];
    edt1d(f, w, d, v, z);
    for (let x = 0; x < w; x++) out[y * w + x] = Math.sqrt(d[x]);
  }
  return out;
}

/**
 * Field for a plan drawing: dark pixels (lines, wall poché) become distance-zero cells.
 * cell size ~2.5 cm. The field follows the drawing's group transform at lookup time.
 */
export function drawingField(r, cellM = 0.025) {
  const f = Math.max(1, Math.floor(r.ppm * cellM)); // drawing pixels per cell
  const gw = Math.ceil(r.pxW / f), gh = Math.ceil(r.pxH / f);
  const mask = new Uint8Array(gw * gh);
  const ctx = r.canvas.getContext("2d", { willReadFrequently: true });
  const strip = Math.max(f, Math.floor(4_000_000 / r.pxW / f) * f); // read in strips to bound memory
  for (let y0 = 0; y0 < r.pxH; y0 += strip) {
    const hh = Math.min(strip, r.pxH - y0);
    const data = ctx.getImageData(0, y0, r.pxW, hh).data;
    for (let y = 0; y < hh; y++) {
      const cy = Math.floor((y0 + y) / f) * gw;
      for (let x = 0; x < r.pxW; x++) {
        const i = (y * r.pxW + x) * 4;
        if (data[i] * 0.299 + data[i + 1] * 0.587 + data[i + 2] * 0.114 < 140) mask[cy + Math.floor(x / f)] = 1;
      }
    }
  }
  const dt = distanceTransform(mask, gw, gh);
  const W = r.pxW / r.ppm, H = r.pxH / r.ppm, cellLocal = f / r.ppm;
  return makeField(dt, gw, gh, r.group, (lx, lz) => [(lx + W / 2) / cellLocal, (lz + H / 2) / cellLocal], cellLocal);
}

/** Field for a BIM model: triangles cut by the horizontal plane y = sliceY (world) give section lines. */
export function bimField(model, sliceY, cellM = 0.025) {
  model.group.updateMatrixWorld(true);
  const inv = model.group.matrixWorld.clone().invert();
  const localY = new THREE.Vector3(0, sliceY, 0).applyMatrix4(inv).y; // models only rotate about Y
  const segs = [];
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
  for (const mesh of model.group.children) {
    if (!mesh.isMesh) continue;
    const pos = mesh.geometry.attributes.position, idx = mesh.geometry.index;
    const tri = idx ? idx.count / 3 : pos.count / 3;
    for (let t = 0; t < tri; t++) {
      const i0 = idx ? idx.getX(t * 3) : t * 3, i1 = idx ? idx.getX(t * 3 + 1) : t * 3 + 1, i2 = idx ? idx.getX(t * 3 + 2) : t * 3 + 2;
      a.fromBufferAttribute(pos, i0); b.fromBufferAttribute(pos, i1); c.fromBufferAttribute(pos, i2);
      const pts = [];
      for (const [p, q] of [[a, b], [b, c], [c, a]]) {
        const dp = p.y - localY, dq = q.y - localY;
        if ((dp < 0) !== (dq < 0)) { const k = dp / (dp - dq); pts.push(p.x + (q.x - p.x) * k, p.z + (q.z - p.z) * k); }
      }
      if (pts.length >= 4) segs.push(pts[0], pts[1], pts[2], pts[3]);
    }
  }
  if (!segs.length) return null;
  let minX = Infinity, minZ = Infinity, maxX = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < segs.length; i += 2) { minX = Math.min(minX, segs[i]); maxX = Math.max(maxX, segs[i]); minZ = Math.min(minZ, segs[i + 1]); maxZ = Math.max(maxZ, segs[i + 1]); }
  minX -= 1; minZ -= 1; maxX += 1; maxZ += 1;
  const scale = model.group.scale.x || 1;
  const cellLocal = cellM / scale;
  const gw = Math.ceil((maxX - minX) / cellLocal), gh = Math.ceil((maxZ - minZ) / cellLocal);
  if (gw * gh > 40_000_000) return null;
  const mask = new Uint8Array(gw * gh);
  for (let i = 0; i < segs.length; i += 4) {
    const x0 = (segs[i] - minX) / cellLocal, z0 = (segs[i + 1] - minZ) / cellLocal, x1 = (segs[i + 2] - minX) / cellLocal, z1 = (segs[i + 3] - minZ) / cellLocal;
    const n = Math.max(1, Math.ceil(Math.hypot(x1 - x0, z1 - z0)));
    for (let k = 0; k <= n; k++) {
      const x = Math.floor(x0 + (x1 - x0) * k / n), z = Math.floor(z0 + (z1 - z0) * k / n);
      if (x >= 0 && z >= 0 && x < gw && z < gh) mask[z * gw + x] = 1;
    }
  }
  const dt = distanceTransform(mask, gw, gh);
  return makeField(dt, gw, gh, model.group, (lx, lz) => [(lx - minX) / cellLocal, (lz - minZ) / cellLocal], cellLocal, segs.length / 4);
}

function makeField(dt, gw, gh, group, localToCell, cellLocal, segments) {
  group.updateMatrixWorld(true);
  const e = group.matrixWorld.clone().invert().elements;
  const metersPerCell = cellLocal * group.scale.x;
  const far = Math.max(gw, gh);
  // world (x, z) -> distance to the nearest line in metres (bilinear)
  const dist = (x, z) => {
    const [u, v] = localToCell(e[0] * x + e[8] * z + e[12], e[2] * x + e[10] * z + e[14]);
    if (!(u >= 0 && v >= 0 && u < gw - 1 && v < gh - 1)) return far * metersPerCell;
    const iu = u | 0, iv = v | 0, fu = u - iu, fv = v - iv, i = iv * gw + iu;
    return ((dt[i] * (1 - fu) + dt[i + 1] * fu) * (1 - fv) + (dt[i + gw] * (1 - fu) + dt[i + gw + 1] * fu) * fv) * metersPerCell;
  };
  // world-space bounds of the field
  const m = group.matrixWorld.elements, cells = [[0, 0], [gw, 0], [0, gh], [gw, gh]];
  const inv = (u, v) => {
    // invert localToCell numerically through two probes (it is affine)
    const [u0, v0] = localToCell(0, 0), [u1] = localToCell(1, 0), [, v1] = localToCell(0, 1);
    return [(u - u0) / (u1 - u0), (v - v0) / (v1 - v0)];
  };
  let minX = Infinity, minZ = Infinity, maxX = -Infinity, maxZ = -Infinity;
  for (const [u, v] of cells) {
    const [lx, lz] = inv(u, v);
    const x = m[0] * lx + m[8] * lz + m[12], z = m[2] * lx + m[10] * lz + m[14];
    minX = Math.min(minX, x); maxX = Math.max(maxX, x); minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);
  }
  return { dist, bounds: { minX, minZ, maxX, maxZ }, segments };
}

// ---------------------------------------------------------------- scan slice
/** wall-height slice of world points, thinned to one point per 2 cm cell, as flat [x, z, ...] */
export function sliceScan(P, y0, y1, maxPts = 6000) {
  const cells = new Map();
  for (let i = 0; i < P.length; i += 3) {
    const y = P[i + 1];
    if (y < y0 || y > y1) continue;
    const key = `${Math.round(P[i] / 0.02)},${Math.round(P[i + 2] / 0.02)}`;
    if (!cells.has(key)) cells.set(key, [P[i], P[i + 2]]);
  }
  let pts = [...cells.values()];
  if (pts.length > maxPts) { const step = pts.length / maxPts; pts = Array.from({ length: maxPts }, (_, i) => pts[Math.floor(i * step)]); }
  return Float64Array.from(pts.flat());
}

// ---------------------------------------------------------------- scoring
function centroid(S) {
  let cx = 0, cz = 0;
  for (let i = 0; i < S.length; i += 2) { cx += S[i]; cz += S[i + 1]; }
  return [cx / (S.length / 2), cz / (S.length / 2)];
}
function cost(field, S, c, pose, tau, step = 1) {
  const cs = Math.cos(pose.phi) * pose.s, sn = Math.sin(pose.phi) * pose.s;
  let sum = 0;
  for (let i = 0; i < S.length; i += 2 * step) {
    const dx = S[i] - c[0], dz = S[i + 1] - c[1];
    const d = field.dist(cs * dx + sn * dz + c[0] + pose.tx, -sn * dx + cs * dz + c[1] + pose.tz);
    sum += Math.min(d, tau) ** 2;
  }
  return sum;
}
export function quality(field, S, c, pose) {
  const cs = Math.cos(pose.phi) * pose.s, sn = Math.sin(pose.phi) * pose.s;
  const ds = [];
  for (let i = 0; i < S.length; i += 2) {
    const dx = S[i] - c[0], dz = S[i + 1] - c[1];
    ds.push(field.dist(cs * dx + sn * dz + c[0] + pose.tx, -sn * dx + cs * dz + c[1] + pose.tz));
  }
  const inl = ds.filter((d) => d < 0.05).sort((a, b) => a - b);
  return { inlier: inl.length / ds.length, median: inl.length ? inl[inl.length >> 1] : null };
}

/** local pattern search; withScale also adjusts uniform scale */
export function refine(field, S, init = { phi: 0, tx: 0, tz: 0, s: 1 }, { withScale = false } = {}) {
  const c = centroid(S);
  const pose = { ...init };
  for (const tau of [0.3, 0.12, 0.05]) {
    let st = { phi: 0.5 * Math.PI / 180, t: tau / 3, s: 0.004 };
    let best = cost(field, S, c, pose, tau);
    for (let iter = 0; iter < 400 && (st.t > 0.0005 || st.phi > 0.0002); iter++) {
      let improved = false;
      const moves = [["phi", st.phi], ["phi", -st.phi], ["tx", st.t], ["tx", -st.t], ["tz", st.t], ["tz", -st.t]];
      if (withScale) moves.push(["s", st.s], ["s", -st.s]);
      for (const [k, dv] of moves) {
        pose[k] += dv;
        const v = cost(field, S, c, pose, tau);
        if (v < best) { best = v; improved = true; } else pose[k] -= dv;
      }
      if (!improved) { st = { phi: st.phi / 2, t: st.t / 2, s: st.s / 2 }; }
    }
  }
  return { pose, c, ...quality(field, S, c, pose) };
}

/**
 * Global search: rotations every 4 degrees x translations on a 25 cm grid covering the field,
 * scored on a subsample, then the best distinct candidates are refined.
 */
export async function globalSearch(field, S, { onProgress, top = 5 } = {}) {
  const c = centroid(S);
  const sub = new Float64Array(Math.min(S.length, 800));
  const step = S.length / sub.length;
  for (let i = 0; i < sub.length; i += 2) { const j = Math.floor(i * step / 2) * 2; sub[i] = S[j]; sub[i + 1] = S[j + 1]; }
  const { minX, minZ, maxX, maxZ } = field.bounds;
  const grid = 0.25, sigma = 0.15, cands = [];
  for (let a = 0; a < 90; a++) {
    const phi = (a * 4 * Math.PI) / 180, cs = Math.cos(phi), sn = Math.sin(phi);
    const rx = new Float64Array(sub.length / 2), rz = new Float64Array(sub.length / 2);
    for (let i = 0, k = 0; i < sub.length; i += 2, k++) { const dx = sub[i] - c[0], dz = sub[i + 1] - c[1]; rx[k] = cs * dx + sn * dz; rz[k] = -sn * dx + cs * dz; }
    let best = { score: -1 };
    for (let X = minX; X <= maxX; X += grid) for (let Z = minZ; Z <= maxZ; Z += grid) {
      let sc = 0;
      for (let k = 0; k < rx.length; k++) { const d = field.dist(rx[k] + X, rz[k] + Z); if (d < 0.45) sc += Math.exp(-(d * d) / (2 * sigma * sigma)); }
      if (sc > best.score) best = { score: sc, phi, tx: X - c[0], tz: Z - c[1] };
    }
    cands.push(best);
    if (onProgress && a % 5 === 0) { onProgress(a / 90); await new Promise((r) => setTimeout(r)); }
  }
  cands.sort((p, q) => q.score - p.score);
  const picked = [];
  for (const cd of cands) {
    if (picked.length >= top) break;
    if (picked.some((p) => Math.abs(angleDiff(p.phi, cd.phi)) < 0.2 && Math.hypot(p.tx - cd.tx, p.tz - cd.tz) < 0.6)) continue;
    picked.push(cd);
  }
  const results = picked.map((p) => refine(field, S, { phi: p.phi, tx: p.tx, tz: p.tz, s: 1 }));
  results.sort((p, q) => q.inlier - p.inlier);
  return results;
}
const angleDiff = (a, b) => Math.atan2(Math.sin(a - b), Math.cos(a - b));

/** pose -> world matrix that moves the scan */
export function poseMatrix({ pose, c }) {
  const C = new THREE.Matrix4().makeTranslation(c[0], 0, c[1]);
  return new THREE.Matrix4().makeTranslation(pose.tx, 0, pose.tz)
    .multiply(C)
    .multiply(new THREE.Matrix4().makeRotationY(pose.phi))
    .multiply(new THREE.Matrix4().makeScale(pose.s, pose.s, pose.s))
    .multiply(C.clone().invert());
}

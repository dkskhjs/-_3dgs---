// Floor detection on splat centres / point-cloud points (world coordinates).
//  - plane:   RANSAC finds the dominant planes; the floor is a large plane that bounds the scan
//             (almost everything lies on one side of it) with the shortest extent along its normal.
//             Works for 3DGS scans in arbitrary orientation (COLMAP Y-down, Z-up, tilted ...).
//  - gravity: for LiDAR-style scans whose axes are already gravity aligned (Y-up or Z-up):
//             the floor is the lowest strong peak in the height histogram.
import * as THREE from "three";

// gravity-aligned exports may be Y-up, Y-down or Z-up depending on the app, so try both signs
const AXES = [[0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]].map((v) => new THREE.Vector3(...v));

/** centres of (at most `max`) reasonably opaque splats, in world space or the mesh's own space */
export function sampleSplats(mesh, total, max = 150_000, world = true) {
  mesh.updateMatrixWorld(true);
  const stride = Math.max(1, Math.ceil(total / max));
  const out = new Float32Array(Math.ceil(total / stride) * 3);
  const m = world ? mesh.matrixWorld.elements : new THREE.Matrix4().elements;
  let k = 0;
  mesh.forEachSplat((i, c, _s, _q, opacity) => {
    if (i % stride || opacity < 0.15 || k >= out.length) return;
    const x = c.x, y = c.y, z = c.z;
    out[k++] = m[0] * x + m[4] * y + m[8] * z + m[12];
    out[k++] = m[1] * x + m[5] * y + m[9] * z + m[13];
    out[k++] = m[2] * x + m[6] * y + m[10] * z + m[14];
  });
  return out.subarray(0, k);
}

/**
 * 3DGS captures surround the real space with a far-away shell of background splats (sky, distant
 * scenery). The core is what lies within 3x the 40th-percentile distance from the median centre,
 * trimmed to its 0.5-99.5 % extent. Returns the core box and the share of splats inside it.
 */
export function computeCore(P) {
  const n = P.length / 3;
  const xs = new Float32Array(n), ys = new Float32Array(n), zs = new Float32Array(n);
  for (let i = 0; i < n; i++) { xs[i] = P[i * 3]; ys[i] = P[i * 3 + 1]; zs[i] = P[i * 3 + 2]; }
  const [cx] = percentiles(xs, [0.5]), [cy] = percentiles(ys, [0.5]), [cz] = percentiles(zs, [0.5]);
  const d = new Float32Array(n);
  for (let i = 0; i < n; i++) d[i] = Math.hypot(xs[i] - cx, ys[i] - cy, zs[i] - cz);
  const [r40] = percentiles(d, [0.4]);
  const R = Math.max(3 * r40, 1e-6);
  const kx = [], ky = [], kz = [];
  for (let i = 0; i < n; i++) if (d[i] <= R) { kx.push(xs[i]); ky.push(ys[i]); kz.push(zs[i]); }
  const [x0, x1] = percentiles(kx, [0.005, 0.995]), [y0, y1] = percentiles(ky, [0.005, 0.995]), [z0, z1] = percentiles(kz, [0.005, 0.995]);
  const box = new THREE.Box3(new THREE.Vector3(x0, y0, z0), new THREE.Vector3(x1, y1, z1));
  box.expandByVector(box.getSize(new THREE.Vector3()).multiplyScalar(0.04));
  let inside = 0;
  const v = new THREE.Vector3();
  for (let i = 0; i < n; i++) if (box.containsPoint(v.set(xs[i], ys[i], zs[i]))) inside++;
  return { box, frac: inside / n };
}

/** points (xyz…) that fall inside an oriented box given as local box + its world matrix */
export function filterToBox(P, box, matrixWorld) {
  const inv = matrixWorld.clone().invert().elements;
  const out = new Float32Array(P.length);
  let k = 0;
  for (let i = 0; i < P.length; i += 3) {
    const x = P[i], y = P[i + 1], z = P[i + 2];
    const lx = inv[0] * x + inv[4] * y + inv[8] * z + inv[12];
    const ly = inv[1] * x + inv[5] * y + inv[9] * z + inv[13];
    const lz = inv[2] * x + inv[6] * y + inv[10] * z + inv[14];
    if (lx >= box.min.x && lx <= box.max.x && ly >= box.min.y && ly <= box.max.y && lz >= box.min.z && lz <= box.max.z) {
      out[k++] = x; out[k++] = y; out[k++] = z;
    }
  }
  return out.subarray(0, k);
}

function percentiles(values, ps) {
  const s = Float32Array.from(values).sort();
  return ps.map((p) => s[Math.min(s.length - 1, Math.max(0, Math.floor(p * s.length)))]);
}

function robustDiag(P) {
  const n = P.length / 3, xs = new Float32Array(n), ys = new Float32Array(n), zs = new Float32Array(n);
  for (let i = 0; i < n; i++) { xs[i] = P[i * 3]; ys[i] = P[i * 3 + 1]; zs[i] = P[i * 3 + 2]; }
  const [x0, x1] = percentiles(xs, [0.02, 0.98]), [y0, y1] = percentiles(ys, [0.02, 0.98]), [z0, z1] = percentiles(zs, [0.02, 0.98]);
  return Math.hypot(x1 - x0, y1 - y0, z1 - z0) || 1;
}

// smallest-eigenvalue eigenvector of the inliers' covariance = best-fit plane normal
function pcaPlane(P, idx) {
  let cx = 0, cy = 0, cz = 0;
  for (const i of idx) { cx += P[i * 3]; cy += P[i * 3 + 1]; cz += P[i * 3 + 2]; }
  const n = idx.length;
  cx /= n; cy /= n; cz /= n;
  let xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
  for (const i of idx) {
    const x = P[i * 3] - cx, y = P[i * 3 + 1] - cy, z = P[i * 3 + 2] - cz;
    xx += x * x; xy += x * y; xz += x * z; yy += y * y; yz += y * z; zz += z * z;
  }
  const tr = xx + yy + zz;
  // power iteration on (tr*I - C): its dominant eigenvector is C's smallest
  const M = [tr - xx, -xy, -xz, -xy, tr - yy, -yz, -xz, -yz, tr - zz];
  let v = [0.3, 0.9, 0.2];
  for (let it = 0; it < 60; it++) {
    const w = [M[0] * v[0] + M[1] * v[1] + M[2] * v[2], M[3] * v[0] + M[4] * v[1] + M[5] * v[2], M[6] * v[0] + M[7] * v[1] + M[8] * v[2]];
    const l = Math.hypot(...w) || 1;
    v = w.map((x) => x / l);
  }
  const normal = new THREE.Vector3(...v).normalize();
  const centroid = new THREE.Vector3(cx, cy, cz);
  return { normal, d: -normal.dot(centroid), centroid };
}

function detectPlanes(P, eps, { maxPlanes = 6, minFrac = 0.02, iters = 600 } = {}) {
  const N = P.length / 3;
  const active = new Uint8Array(N).fill(1);
  const planes = [];
  let seed = 12345;
  const rnd = () => ((seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3(), n = new THREE.Vector3();
  for (let pl = 0; pl < maxPlanes; pl++) {
    const act = [];
    for (let i = 0; i < N; i++) if (active[i]) act.push(i);
    if (act.length < Math.max(50, minFrac * N)) break;
    const sub = [];
    const subN = Math.min(act.length, 12000);
    for (let j = 0; j < subN; j++) sub.push(act[Math.floor(rnd() * act.length)]);
    let best = null, bestCount = 0;
    for (let it = 0; it < iters; it++) {
      const i0 = act[Math.floor(rnd() * act.length)], i1 = act[Math.floor(rnd() * act.length)], i2 = act[Math.floor(rnd() * act.length)];
      a.fromArray(P, i0 * 3); b.fromArray(P, i1 * 3); c.fromArray(P, i2 * 3);
      n.subVectors(b, a).cross(c.sub(a));
      const len = n.length();
      if (len < eps * eps) continue; // degenerate / tiny triangle
      n.divideScalar(len);
      const d = -n.dot(a);
      let cnt = 0;
      for (const i of sub) if (Math.abs(n.x * P[i * 3] + n.y * P[i * 3 + 1] + n.z * P[i * 3 + 2] + d) < eps) cnt++;
      if (cnt > bestCount) { bestCount = cnt; best = { normal: n.clone(), d }; }
    }
    if (!best) break;
    // refine twice with PCA over every active point
    let plane = best, inl = [];
    for (let r = 0; r < 2; r++) {
      inl = [];
      const { normal: q, d } = plane;
      for (const i of act) if (Math.abs(q.x * P[i * 3] + q.y * P[i * 3 + 1] + q.z * P[i * 3 + 2] + d) < eps) inl.push(i);
      if (inl.length < 3) break;
      plane = pcaPlane(P, inl);
    }
    if (inl.length < minFrac * N) break;
    for (const i of inl) active[i] = 0;
    planes.push({ ...plane, count: inl.length, frac: inl.length / N });
  }
  return planes;
}

function orientationPrior(up) {
  // common conventions: Y-up (three.js / glTF / phone apps), Y-down (COLMAP 3DGS), Z-up (CAD / LiDAR)
  if (up.y > 0.9) return 1.5;
  if (up.y < -0.9) return 1.35;
  if (up.z > 0.9) return 1.3;
  return 1;
}

function planeCandidates(P, eps) {
  const planes = detectPlanes(P, eps);
  const N = P.length / 3;
  const step = Math.max(1, Math.floor(N / 60000));
  const cands = [];
  for (const pl of planes) {
    for (const sgn of [1, -1]) {
      const up = pl.normal.clone().multiplyScalar(sgn), d = pl.d * sgn;
      let above = 0, cnt = 0;
      const proj = [];
      for (let i = 0; i < N; i += step) {
        const h = up.x * P[i * 3] + up.y * P[i * 3 + 1] + up.z * P[i * 3 + 2] + d;
        if (h > -3 * eps) above++;
        proj.push(h);
        cnt++;
      }
      const aboveFrac = above / cnt;
      const [p01, p99] = percentiles(proj, [0.01, 0.99]);
      cands.push({ method: "plane", up, d, point: pl.centroid.clone().addScaledVector(up, -(up.dot(pl.centroid) + d)), frac: pl.frac, aboveFrac, extent: Math.max(eps, p99 - p01) });
    }
  }
  // bounding planes only compete on extent with other bounding planes
  const bounding = cands.filter((c) => c.aboveFrac > 0.85);
  const minExt = Math.min(...(bounding.length ? bounding : cands).map((c) => c.extent));
  for (const c of cands) c.score = c.frac * c.aboveFrac ** 6 * orientationPrior(c.up) * Math.sqrt(minExt / c.extent);
  cands.sort((x, y) => y.score - x.score);
  // keep the best sign per plane, drop near-duplicates
  const out = [];
  for (const c of cands) if (!out.some((o) => o.up.angleTo(c.up) < 0.1 || (o.up.angleTo(c.up) > Math.PI - 0.1 && Math.abs(o.up.dot(o.point) - o.up.dot(c.point)) < 4 * eps))) out.push(c);
  return out;
}

function gravityCandidates(P) {
  const N = P.length / 3;
  const out = [];
  for (const up of AXES) {
    const proj = new Float32Array(N);
    for (let i = 0; i < N; i++) proj[i] = up.x * P[i * 3] + up.y * P[i * 3 + 1] + up.z * P[i * 3 + 2];
    const [lo, hi] = percentiles(proj, [0.005, 0.995]);
    const range = hi - lo || 1;
    const bins = 300, bw = range / bins, hist = new Uint32Array(bins);
    for (const h of proj) { const b = Math.floor((h - lo) / bw); if (b >= 0 && b < bins) hist[b]++; }
    let max = 0;
    for (const c of hist) max = Math.max(max, c);
    // lowest strong bin in the bottom 40 %, then climb to its local peak
    let fb = -1;
    for (let b = 0; b < bins * 0.4; b++) if (hist[b] >= 0.3 * max) { fb = b; break; }
    if (fb < 0) continue;
    while (fb + 1 < bins && hist[fb + 1] > hist[fb]) fb++;
    const nz = [...hist].filter((c) => c > 0).sort((a, b) => a - b);
    const quality = hist[fb] / (nz[nz.length >> 1] || 1);
    // floor height = mean of points within one bin of the peak
    let s = 0, k = 0;
    const h0 = lo + (fb - 1) * bw, h1 = lo + (fb + 2) * bw;
    for (const h of proj) if (h >= h0 && h <= h1) { s += h; k++; }
    const off = s / k;
    // a real floor has (almost) everything above it; the wrong sign usually finds the ceiling or nothing
    let above = 0;
    for (const h of proj) if (h >= off - 2 * bw) above++;
    const aboveFrac = above / N;
    out.push({ method: "gravity", up: up.clone(), d: -off, point: up.clone().multiplyScalar(off), frac: k / N, aboveFrac, extent: range, quality, score: quality * aboveFrac ** 6 });
  }
  return out.sort((a, b) => b.score - a.score);
}

/**
 * @param {Float32Array} P world points (xyz…)
 * @param {"scaniverse"|"auto"|"plane"|"gravity"} mode  scaniverse = gravity first (LiDAR, metric, gravity aligned)
 * @param {boolean} preferGravity hint for auto mode (true for LiDAR / point-cloud PLYs)
 * @returns candidates, best first: {method, up:Vector3, point:Vector3, frac, ...}
 */
export function detectFloor(P, mode = "auto", preferGravity = false) {
  if (P.length < 300) return [];
  const eps = 0.006 * robustDiag(P);
  if (mode === "gravity") return gravityCandidates(P);
  if (mode === "plane") return planeCandidates(P, eps);
  const grav = gravityCandidates(P), planes = planeCandidates(P, eps);
  const gravGood = grav[0] && grav[0].quality > 4 && grav[0].aboveFrac > 0.9;
  return (preferGravity || mode === "scaniverse") && gravGood ? [...grav, ...planes] : [...planes, ...grav];
}

/** world transform that puts the candidate floor on y = 0 with +Y up */
export function floorTransform(c) {
  const q = new THREE.Quaternion().setFromUnitVectors(c.up.clone().normalize(), new THREE.Vector3(0, 1, 0));
  const R = new THREE.Matrix4().makeRotationFromQuaternion(q);
  const p = c.point.clone().applyMatrix4(R);
  return new THREE.Matrix4().makeTranslation(0, -p.y, 0).multiply(R);
}

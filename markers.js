// Colour markers in a scan (fire-hose cabinets, red dots, coloured targets) and order-free matching.
//  detectMarkers: splats close to a target hue -> connected clusters on a voxel grid -> centroids
//  matchMarkers:  RANSAC over marker pairs; consistent distances give the transform without knowing
//                 which marker is which (a stand-in for printed QR targets)
import * as THREE from "three";

export function hsv(r, g, b) {
  const mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
  let h = 0;
  if (d > 1e-6) {
    if (mx === r) h = ((g - b) / d) % 6;
    else if (mx === g) h = (b - r) / d + 2;
    else h = (r - g) / d + 4;
    h *= 60;
    if (h < 0) h += 360;
  }
  return [h, mx > 0 ? d / mx : 0, mx];
}

/**
 * @param mesh SplatMesh; positions are returned in the mesh's own space so they follow later moves
 * @param opts {hue (deg), hueTol, minSat, minVal, cell, minCount, maxSize, box (local Box3, optional)}
 */
export function detectMarkers(mesh, total, opts) {
  const { hue, hueTol, minSat, minVal, cell, minCount, maxSize, box } = opts;
  const stride = Math.max(1, Math.ceil(total / 8_000_000));
  const pts = [];
  mesh.forEachSplat((i, c, _s, _q, opacity, color) => {
    if (i % stride || opacity < 0.3) return;
    const [h, s, v] = hsv(color.r, color.g, color.b);
    const dh = Math.min(Math.abs(h - hue), 360 - Math.abs(h - hue));
    if (dh > hueTol || s < minSat || v < minVal) return;
    if (box && !box.containsPoint(c)) return;
    pts.push(c.x, c.y, c.z);
  });
  // connected components over a voxel grid (26-neighbourhood)
  const grid = new Map();
  const key = (a, b, c) => `${a},${b},${c}`;
  for (let i = 0; i < pts.length; i += 3) {
    const k = key(Math.floor(pts[i] / cell), Math.floor(pts[i + 1] / cell), Math.floor(pts[i + 2] / cell));
    let arr = grid.get(k);
    if (!arr) grid.set(k, (arr = []));
    arr.push(i);
  }
  const seen = new Set(), markers = [];
  for (const start of grid.keys()) {
    if (seen.has(start)) continue;
    const stack = [start], members = [];
    seen.add(start);
    while (stack.length) {
      const k = stack.pop();
      members.push(...grid.get(k));
      const [a, b, c] = k.split(",").map(Number);
      for (let i = -1; i <= 1; i++) for (let j = -1; j <= 1; j++) for (let l = -1; l <= 1; l++) {
        const n = key(a + i, b + j, c + l);
        if (grid.has(n) && !seen.has(n)) { seen.add(n); stack.push(n); }
      }
    }
    if (members.length < minCount) continue;
    const bb = new THREE.Box3(), ctr = new THREE.Vector3(), v = new THREE.Vector3();
    for (const i of members) { v.set(pts[i], pts[i + 1], pts[i + 2]); bb.expandByPoint(v); ctr.add(v); }
    ctr.divideScalar(members.length);
    const size = bb.getSize(new THREE.Vector3());
    if (Math.max(size.x, size.y, size.z) > maxSize) continue;
    markers.push({ local: ctr, count: members.length, size: Math.max(size.x, size.y, size.z) });
  }
  markers.sort((a, b) => b.count - a.count);
  return markers.slice(0, 40);
}

/**
 * Order-free matching of src points onto dst points (world Vector3s).
 * solve(P, Q, opts) is the up-axis constrained similarity solver; it returns {M, s, theta, residuals, rms}.
 * tol: inlier distance after the transform. Returns null when fewer than 2 pairs agree.
 */
export function matchMarkers(src, dst, solve, { withScale, planar, tol }) {
  const d2 = (a, b) => (planar ? Math.hypot(a.x - b.x, a.z - b.z) : a.distanceTo(b));
  let best = null;
  for (let i = 0; i < dst.length; i++) for (let j = i + 1; j < dst.length; j++) {
    const dRef = d2(dst[i], dst[j]);
    if (dRef < tol * 2) continue;
    for (let a = 0; a < src.length; a++) for (let b = 0; b < src.length; b++) {
      if (a === b) continue;
      const dSrc = d2(src[a], src[b]);
      if (!withScale && Math.abs(dRef - dSrc) > Math.max(tol, 0.05 * dRef)) continue;
      if (withScale && (dSrc < 1e-6 || dRef / dSrc > 50 || dSrc / dRef > 50)) continue;
      const { M } = solve([src[a], src[b]], [dst[i], dst[j]], { withScale, planar });
      // greedy one-to-one inliers
      const used = new Set(), pairs = [];
      let err = 0;
      for (let k = 0; k < dst.length; k++) {
        let bi = -1, bd = tol;
        for (let m = 0; m < src.length; m++) {
          if (used.has(m)) continue;
          const d = d2(src[m].clone().applyMatrix4(M), dst[k]);
          if (d < bd) { bd = d; bi = m; }
        }
        if (bi >= 0) { used.add(bi); pairs.push([bi, k]); err += bd; }
      }
      if (!best || pairs.length > best.pairs.length || (pairs.length === best.pairs.length && err < best.err)) best = { pairs, err };
    }
  }
  if (!best || best.pairs.length < 2) return null;
  let pairs = best.pairs;
  let fit = solve(pairs.map(([m]) => src[m]), pairs.map(([, k]) => dst[k]), { withScale, planar });
  // grow / re-pair with the least-squares fit until stable, so every marker that can match is used
  for (let it = 0; it < 5; it++) {
    const used = new Set(), next = [];
    for (let k = 0; k < dst.length; k++) {
      let bi = -1, bd = tol * 1.5;
      for (let m = 0; m < src.length; m++) {
        if (used.has(m)) continue;
        const d = d2(src[m].clone().applyMatrix4(fit.M), dst[k]);
        if (d < bd) { bd = d; bi = m; }
      }
      if (bi >= 0) { used.add(bi); next.push([bi, k]); }
    }
    if (next.length < pairs.length) break;
    const same = next.length === pairs.length && next.every(([m, k], i) => pairs[i][0] === m && pairs[i][1] === k);
    pairs = next;
    fit = solve(pairs.map(([m]) => src[m]), pairs.map(([, k]) => dst[k]), { withScale, planar });
    if (same) break;
  }
  return { ...fit, pairs };
}


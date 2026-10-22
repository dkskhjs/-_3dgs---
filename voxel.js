// Voxel ("block") scan-to-BIM: occupancy grid of the scan points -> few large boxes.
// The grid is turned to the building's main direction so boxes line up with the walls.
//  plan: 2.5D — a plan cell is boundary when it is filled over most of the floor-to-ceiling height
//        (furniture and ceiling fixtures drop out); rectangles merged greedily, full-height boxes
//  3d:   every occupied voxel, merged greedily into boxes along u, then v, then height

/**
 * @param P      world points [x, y, z, ...] (core of the scan)
 * @param opts   {floorY, ceilY (top of the scanned space), angle (rad, main direction in x/z), size (m),
 *               mode "height"|"plan"|"3d", minRun (m of vertical fill that makes a plan cell a boundary),
 *               floorBand (m above the floor treated as the floor itself), ceilCut (y above which points are
 *               the ceiling itself; optional)}
 * @returns boxes in world terms: {a:[x,z], b:[x,z], t, y0, h} (a centre line, thickness, base, height)
 */
export function voxelBoxes(P, { floorY, ceilY, angle = 0, size = 0.2, mode = "height", minRun = 0.8, floorBand = 0.06, ceilCut = Infinity }) {
  const c = Math.cos(angle), s = Math.sin(angle);
  const height = ceilY - floorY;
  if (!(height > 0)) return { boxes: [], cells: 0 };
  // into the grid frame: u along the main direction, v across it
  let u0 = Infinity, u1 = -Infinity, v0 = Infinity, v1 = -Infinity;
  const U = new Float32Array(P.length / 3), V = new Float32Array(P.length / 3), Y = new Float32Array(P.length / 3);
  let n = 0;
  for (let i = 0; i < P.length; i += 3) {
    const y = P[i + 1];
    // the floor and ceiling surfaces themselves are not objects
    if (y < floorY + floorBand || y > Math.min(ceilY, ceilCut)) continue;
    const u = P[i] * c + P[i + 2] * s, v = -P[i] * s + P[i + 2] * c;
    U[n] = u; V[n] = v; Y[n] = y; n++;
    u0 = Math.min(u0, u); u1 = Math.max(u1, u); v0 = Math.min(v0, v); v1 = Math.max(v1, v);
  }
  if (!n) return { boxes: [], cells: 0 };
  const nu = Math.ceil((u1 - u0) / size) + 1, nv = Math.ceil((v1 - v0) / size) + 1, ny = Math.max(1, Math.ceil(height / size));
  if (nu * nv * ny > 30_000_000) throw new Error("블록 크기가 너무 작습니다 — 더 큰 크기를 고르세요");
  const count = new Uint16Array(nu * nv * ny);
  const at = (iu, iv, iy) => (iy * nv + iv) * nu + iu;
  for (let k = 0; k < n; k++) {
    const iu = Math.floor((U[k] - u0) / size), iv = Math.floor((V[k] - v0) / size), iy = Math.min(ny - 1, Math.floor((Y[k] - floorY) / size));
    const i = at(iu, iv, iy);
    if (count[i] < 65535) count[i]++;
  }
  // occupied = clearly denser than stray splats: 30 % of the median non-empty voxel
  const nz = [];
  for (let i = 0; i < count.length; i += Math.max(1, Math.floor(count.length / 200000))) if (count[i]) nz.push(count[i]);
  nz.sort((a, b) => a - b);
  const thr = Math.max(2, Math.round((nz[nz.length >> 1] || 1) * 0.3));
  const occ = (iu, iv, iy) => count[at(iu, iv, iy)] >= thr;

  const toWorld = (u, v) => [u * c - v * s, u * s + v * c];
  const boxOut = (bu0, bu1, bv0, bv1, y0, y1) => {
    // a box becomes a "wall": centre line along its longer side, thickness = shorter side
    const U0 = u0 + bu0 * size, U1 = u0 + bu1 * size, V0 = v0 + bv0 * size, V1 = v0 + bv1 * size;
    const alongU = U1 - U0 >= V1 - V0;
    const a = alongU ? toWorld(U0, (V0 + V1) / 2) : toWorld((U0 + U1) / 2, V0);
    const b = alongU ? toWorld(U1, (V0 + V1) / 2) : toWorld((U0 + U1) / 2, V1);
    return { a, b, t: alongU ? V1 - V0 : U1 - U0, y0, h: y1 - y0 };
  };

  if (mode === "height") {
    // height map: every plan cell gets one column from its lowest to its highest filled voxel;
    // columns starting within 30 cm of the floor stand on it. Cells with the same column merge.
    const key = new Int32Array(nu * nv); // 0 = empty, else (bottom layer + 1) * 4096 + top layer
    const floorSnap = Math.floor(0.3 / size);
    for (let iv = 0; iv < nv; iv++) for (let iu = 0; iu < nu; iu++) {
      let bottom = -1, top = -1;
      for (let iy = 0; iy < ny; iy++) if (occ(iu, iv, iy)) { if (bottom < 0) bottom = iy; top = iy; }
      if (bottom < 0) continue;
      if (bottom <= floorSnap) bottom = 0;
      key[iv * nu + iu] = (bottom + 1) * 4096 + top;
    }
    // drop specks: a column with no filled neighbour at all
    const keep = new Uint8Array(nu * nv);
    for (let iv = 0; iv < nv; iv++) for (let iu = 0; iu < nu; iu++) {
      if (!key[iv * nu + iu]) continue;
      let nb = 0;
      for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) {
        const x = iu + a, z = iv + b;
        if ((a || b) && x >= 0 && z >= 0 && x < nu && z < nv && key[z * nu + x]) nb++;
      }
      if (nb) keep[iv * nu + iu] = 1;
    }
    for (let i = 0; i < key.length; i++) if (!keep[i]) key[i] = 0;
    const rects = greedyKeyed(key, nu, nv);
    const boxes = rects.map(([a, b, c2, d, k]) => {
      const bottom = Math.floor(k / 4096) - 1, top = k % 4096;
      return boxOut(a, b, c2, d, floorY + (bottom ? bottom * size : 0), Math.min(ceilY, floorY + (top + 1) * size));
    });
    return { boxes, cells: keep.reduce((x, y) => x + y, 0) };
  }

  if (mode === "plan") {
    // a boundary cell has at least `minRun` metres of filled voxels above 30 cm (clutter on the floor
    // and a thin ceiling layer do not qualify); no assumption about one ceiling height for the room
    const lo = Math.min(ny - 1, Math.floor(0.3 / size));
    const need = Math.max(2, Math.round(minRun / size));
    let grid = new Uint8Array(nu * nv);
    for (let iv = 0; iv < nv; iv++) for (let iu = 0; iu < nu; iu++) {
      let f = 0;
      for (let iy = lo; iy < ny; iy++) if (occ(iu, iv, iy)) f++;
      if (f >= need) grid[iv * nu + iu] = 1;
    }
    grid = cleanup2d(grid, nu, nv);
    const rects = greedy2d(grid, nu, nv);
    // each box takes the height the scan actually has over it: lowest to highest layer where at least
    // 30 % of its cells are filled; boxes starting within 50 cm of the floor stand on the floor
    const boxes = rects.map(([a, b, c2, d]) => {
      const cells = (b - a) * (d - c2);
      let yLo = -1, yHi = -1;
      for (let iy = 0; iy < ny; iy++) {
        let f = 0;
        for (let iv = c2; iv < d; iv++) for (let iu = a; iu < b; iu++) if (occ(iu, iv, iy)) f++;
        if (f >= Math.max(1, 0.3 * cells)) { if (yLo < 0) yLo = iy; yHi = iy; }
      }
      if (yLo < 0) { yLo = lo; yHi = ny - 1; }
      const y0 = yLo * size <= 0.5 ? floorY : floorY + yLo * size;
      return boxOut(a, b, c2, d, y0, Math.min(ceilY, floorY + (yHi + 1) * size));
    });
    return { boxes, cells: grid.reduce((x, y) => x + y, 0) };
  }

  // 3d: drop isolated voxels, then merge
  const vox = new Uint8Array(nu * nv * ny);
  for (let iy = 0; iy < ny; iy++) for (let iv = 0; iv < nv; iv++) for (let iu = 0; iu < nu; iu++) {
    if (!occ(iu, iv, iy)) continue;
    let nb = 0;
    for (let a = -1; a <= 1 && nb < 2; a++) for (let b = -1; b <= 1; b++) for (let d = -1; d <= 1; d++) {
      if (!a && !b && !d) continue;
      const x = iu + a, z = iv + b, y = iy + d;
      if (x >= 0 && z >= 0 && y >= 0 && x < nu && z < nv && y < ny && occ(x, z, y)) nb++;
    }
    if (nb >= 2) vox[at(iu, iv, iy)] = 1;
  }
  const boxes = greedy3d(vox, nu, nv, ny, at).map(([a, b, c2, d, e, f]) => boxOut(a, b, c2, d, floorY + e * size, floorY + f * size));
  return { boxes, cells: vox.reduce((x, y) => x + y, 0) };
}

// remove specks (no filled 8-neighbour) and fill pinholes (filled on all 4 sides)
function cleanup2d(g, nu, nv) {
  const out = g.slice();
  const get = (u, v) => (u >= 0 && v >= 0 && u < nu && v < nv ? g[v * nu + u] : 0);
  for (let v = 0; v < nv; v++) for (let u = 0; u < nu; u++) {
    const i = v * nu + u;
    let n8 = 0;
    for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) if ((a || b) && get(u + a, v + b)) n8++;
    if (g[i] && n8 === 0) out[i] = 0;
    if (!g[i] && get(u - 1, v) && get(u + 1, v) && get(u, v - 1) && get(u, v + 1)) out[i] = 1;
  }
  return out;
}

// greedy rectangle cover: grow along u, then along v while the whole row stays filled
function greedy2d(g, nu, nv) {
  const done = new Uint8Array(nu * nv), rects = [];
  for (let v = 0; v < nv; v++) for (let u = 0; u < nu; u++) {
    const i = v * nu + u;
    if (!g[i] || done[i]) continue;
    let w = 1;
    while (u + w < nu && g[i + w] && !done[i + w]) w++;
    let h = 1;
    grow: while (v + h < nv) {
      for (let k = 0; k < w; k++) { const j = (v + h) * nu + u + k; if (!g[j] || done[j]) break grow; }
      h++;
    }
    for (let a = 0; a < h; a++) for (let k = 0; k < w; k++) done[(v + a) * nu + u + k] = 1;
    rects.push([u, u + w, v, v + h]);
  }
  return mergeTouching(rects);
}

// columns are "the same" when they start on the same layer and their tops are within one layer
const sameColumn = (a, b) => a && b && Math.floor(a / 4096) === Math.floor(b / 4096) && Math.abs((a % 4096) - (b % 4096)) <= 1;
const tallest = (a, b) => (a % 4096 >= b % 4096 ? a : b);

// greedy rectangles over cells with matching columns; returns [u0, u1, v0, v1, key] (key = tallest)
function greedyKeyed(key, nu, nv) {
  const done = new Uint8Array(nu * nv), rects = [];
  for (let v = 0; v < nv; v++) for (let u = 0; u < nu; u++) {
    const i = v * nu + u, k0 = key[i];
    if (!k0 || done[i]) continue;
    let k = k0, w = 1;
    while (u + w < nu && sameColumn(key[i + w], k0) && !done[i + w]) { k = tallest(k, key[i + w]); w++; }
    let h = 1;
    grow: while (v + h < nv) {
      let kr = k;
      for (let x = 0; x < w; x++) { const j = (v + h) * nu + u + x; if (!sameColumn(key[j], k0) || done[j]) break grow; kr = tallest(kr, key[j]); }
      k = kr;
      h++;
    }
    for (let a = 0; a < h; a++) for (let x = 0; x < w; x++) done[(v + a) * nu + u + x] = 1;
    rects.push([u, u + w, v, v + h, k]);
  }
  return mergeTouching(rects);
}

// join rectangles that share a whole edge (same span, touching), until nothing changes
function mergeTouching(rects) {
  let merged = true;
  while (merged) {
    merged = false;
    for (let i = 0; i < rects.length && !merged; i++) for (let j = i + 1; j < rects.length; j++) {
      const [a0, a1, b0, b1, ka0] = rects[i], [c0, c1, d0, d1, kb] = rects[j];
      if (ka0 !== kb && !sameColumn(ka0, kb)) continue;
      const ka = ka0 === undefined ? undefined : tallest(ka0, kb);
      if (a0 === c0 && a1 === c1 && (b1 === d0 || d1 === b0)) rects[i] = [a0, a1, Math.min(b0, d0), Math.max(b1, d1), ka];
      else if (b0 === d0 && b1 === d1 && (a1 === c0 || c1 === a0)) rects[i] = [Math.min(a0, c0), Math.max(a1, c1), b0, b1, ka];
      else continue;
      rects.splice(j, 1);
      merged = true;
      break;
    }
  }
  return rects;
}

function greedy3d(g, nu, nv, ny, at) {
  const done = new Uint8Array(g.length), boxes = [];
  const free = (u, v, y) => g[at(u, v, y)] && !done[at(u, v, y)];
  for (let y = 0; y < ny; y++) for (let v = 0; v < nv; v++) for (let u = 0; u < nu; u++) {
    if (!free(u, v, y)) continue;
    let w = 1;
    while (u + w < nu && free(u + w, v, y)) w++;
    let d = 1;
    growV: while (v + d < nv) { for (let k = 0; k < w; k++) if (!free(u + k, v + d, y)) break growV; d++; }
    let h = 1;
    growY: while (y + h < ny) { for (let b = 0; b < d; b++) for (let k = 0; k < w; k++) if (!free(u + k, v + b, y + h)) break growY; h++; }
    for (let c = 0; c < h; c++) for (let b = 0; b < d; b++) for (let k = 0; k < w; k++) done[at(u + k, v + b, y + c)] = 1;
    boxes.push([u, u + w, v, v + d, y, y + h]);
  }
  return boxes;
}

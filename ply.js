// PLY reader for scans that are NOT 3D Gaussian Splatting files: LiDAR / photogrammetry point clouds
// (ascii or binary, float or double coordinates, optional colours, normals, intensity).
// 3DGS PLYs (and SuperSplat compressed PLYs) are left to Spark.

const TYPES = {
  char: ["getInt8", 1], int8: ["getInt8", 1], uchar: ["getUint8", 1], uint8: ["getUint8", 1],
  short: ["getInt16", 2], int16: ["getInt16", 2], ushort: ["getUint16", 2], uint16: ["getUint16", 2],
  int: ["getInt32", 4], int32: ["getInt32", 4], uint: ["getUint32", 4], uint32: ["getUint32", 4],
  float: ["getFloat32", 4], float32: ["getFloat32", 4], double: ["getFloat64", 8], float64: ["getFloat64", 8],
};

export function parsePlyHeader(bytes) {
  const head = new TextDecoder("latin1").decode(bytes.subarray(0, Math.min(bytes.length, 64 * 1024)));
  if (!head.startsWith("ply")) return null;
  const end = head.indexOf("end_header");
  if (end < 0) return null;
  let headerLen = end + "end_header".length;
  if (head[headerLen] === "\r") headerLen++;
  if (head[headerLen] === "\n") headerLen++;
  const h = { format: "", elements: [], headerLen };
  for (const raw of head.slice(0, end).split(/\r?\n/)) {
    const t = raw.trim().split(/\s+/);
    if (t[0] === "format") h.format = t[1];
    else if (t[0] === "element") h.elements.push({ name: t[1], count: parseInt(t[2], 10), props: [] });
    else if (t[0] === "property") {
      const el = h.elements.at(-1);
      if (t[1] === "list") el.props.push({ name: t[4], list: true, countType: t[2], type: t[3] });
      else el.props.push({ name: t[2], type: t[1] });
    }
  }
  return h;
}

/** true when Spark should load the file (3DGS or SuperSplat compressed PLY) */
export function isGaussianPly(h) {
  if (h.elements.some((e) => e.name === "chunk")) return true;
  const v = h.elements.find((e) => e.name === "vertex");
  const names = new Set(v?.props.map((p) => p.name));
  return names.has("scale_0") && names.has("rot_0") && names.has("opacity");
}

/**
 * Reads the vertex element into typed arrays.
 * Coordinates are re-centred on their first point when they are large (e.g. UTM) to keep float32 precision;
 * the removed offset is returned so it can be reported.
 */
export function readPointCloud(bytes, h, { maxPoints = 6_000_000 } = {}) {
  const vi = h.elements.findIndex((e) => e.name === "vertex");
  if (vi < 0) throw new Error("PLY에 vertex 요소가 없습니다");
  const v = h.elements[vi];
  const idx = Object.fromEntries(v.props.map((p, i) => [p.name.toLowerCase(), i]));
  const pick = (...names) => names.map((n) => idx[n]).find((i) => i !== undefined);
  const ix = pick("x"), iy = pick("y"), iz = pick("z");
  if (ix === undefined || iy === undefined || iz === undefined) throw new Error("PLY에 x / y / z 좌표가 없습니다");
  const ir = pick("red", "diffuse_red", "r"), ig = pick("green", "diffuse_green", "g"), ib = pick("blue", "diffuse_blue", "b");
  const iint = pick("intensity", "scalar_intensity", "scalar_intensities", "reflectance");
  const inx = pick("nx", "normal_x"), iny = pick("ny", "normal_y"), inz = pick("nz", "normal_z");

  const total = v.count;
  const stride = Math.max(1, Math.ceil(total / maxPoints));
  const n = Math.ceil(total / stride);
  const pos = new Float32Array(n * 3);
  const col = ir !== undefined && ig !== undefined && ib !== undefined ? new Float32Array(n * 3) : null;
  const inten = !col && iint !== undefined ? new Float32Array(n) : null;
  const nrm = inx !== undefined && iny !== undefined && inz !== undefined ? new Float32Array(n * 3) : null;
  let offset = null;
  const vals = new Float64Array(v.props.length);
  let k = 0;
  const store = (row) => {
    if (row % stride) return;
    let x = vals[ix], y = vals[iy], z = vals[iz];
    if (!Number.isFinite(x + y + z)) return;
    if (!offset) offset = Math.max(Math.abs(x), Math.abs(y), Math.abs(z)) > 1e4 ? [x, y, z] : [0, 0, 0];
    pos[k * 3] = x - offset[0]; pos[k * 3 + 1] = y - offset[1]; pos[k * 3 + 2] = z - offset[2];
    if (col) { col[k * 3] = vals[ir]; col[k * 3 + 1] = vals[ig]; col[k * 3 + 2] = vals[ib]; }
    if (inten) inten[k] = vals[iint];
    if (nrm) { nrm[k * 3] = vals[inx]; nrm[k * 3 + 1] = vals[iny]; nrm[k * 3 + 2] = vals[inz]; }
    k++;
  };

  if (h.format === "ascii") {
    const text = new TextDecoder().decode(bytes.subarray(h.headerLen));
    let p = 0;
    // skip elements that come before the vertices (one line each)
    for (let e = 0; e < vi; e++) for (let i = 0; i < h.elements[e].count; i++) p = text.indexOf("\n", p) + 1;
    for (let row = 0; row < total; row++) {
      let nl = text.indexOf("\n", p);
      if (nl < 0) nl = text.length;
      const t = text.slice(p, nl).trim().split(/\s+/);
      p = nl + 1;
      // scalar properties only; list properties in vertices are not used
      for (let i = 0; i < v.props.length; i++) vals[i] = parseFloat(t[i]);
      store(row);
    }
  } else {
    const le = h.format === "binary_little_endian";
    if (!le && h.format !== "binary_big_endian") throw new Error(`지원하지 않는 PLY 형식: ${h.format}`);
    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    let o = h.headerLen;
    const readEl = (el, onRow) => {
      for (let row = 0; row < el.count; row++) {
        for (let i = 0; i < el.props.length; i++) {
          const p = el.props[i];
          if (p.list) {
            const [cg, cs] = TYPES[p.countType];
            const cnt = dv[cg](o, le);
            o += cs + cnt * TYPES[p.type][1];
            continue;
          }
          const [g, size] = TYPES[p.type] || [];
          if (!g) throw new Error(`알 수 없는 PLY 타입: ${p.type}`);
          if (onRow) vals[i] = dv[g](o, le);
          o += size;
        }
        if (onRow) onRow(row);
      }
    };
    for (let e = 0; e < vi; e++) readEl(h.elements[e], null);
    readEl(v, store);
  }

  // normalise colours to 0..1 whatever their storage type
  if (col) {
    let mx = 0;
    for (let i = 0; i < k * 3; i++) mx = Math.max(mx, col[i]);
    const s = mx > 255 ? 1 / 65535 : mx > 1.0001 ? 1 / 255 : 1;
    for (let i = 0; i < k * 3; i++) col[i] *= s;
  }
  if (inten) {
    const sorted = Float32Array.from(inten.subarray(0, k)).sort();
    const lo = sorted[Math.floor(k * 0.02)], hi = sorted[Math.floor(k * 0.98)] || lo + 1;
    for (let i = 0; i < k; i++) inten[i] = Math.min(1, Math.max(0, (inten[i] - lo) / (hi - lo)));
  }
  let hasNormals = false;
  if (nrm) for (let i = 0; i < Math.min(k, 1000) * 3; i++) if (nrm[i] !== 0) { hasNormals = true; break; }
  return {
    count: k, total, positions: pos.subarray(0, k * 3), colors: col?.subarray(0, k * 3) || null,
    intensity: inten?.subarray(0, k) || null, normals: hasNormals ? nrm.subarray(0, k * 3) : null, offset: offset || [0, 0, 0],
  };
}

/** median nearest-neighbour distance from a spatial hash, used to size point splats */
export function estimateSpacing(pos, count) {
  const m = Math.min(count, 300_000), step = count / m;
  let minX = Infinity, minY = Infinity, minZ = Infinity, maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let j = 0; j < m; j++) {
    const i = Math.floor(j * step) * 3;
    minX = Math.min(minX, pos[i]); maxX = Math.max(maxX, pos[i]);
    minY = Math.min(minY, pos[i + 1]); maxY = Math.max(maxY, pos[i + 1]);
    minZ = Math.min(minZ, pos[i + 2]); maxZ = Math.max(maxZ, pos[i + 2]);
  }
  const diag = Math.hypot(maxX - minX, maxY - minY, maxZ - minZ) || 1;
  let cell = diag / Math.sqrt(m); // points lie on surfaces, so spacing ~ sqrt(area / n)
  for (let attempt = 0; attempt < 4; attempt++) {
    const grid = new Map();
    const key = (a, b, c) => `${a},${b},${c}`;
    for (let j = 0; j < m; j++) {
      const i = Math.floor(j * step) * 3;
      const kk = key(Math.floor(pos[i] / cell), Math.floor(pos[i + 1] / cell), Math.floor(pos[i + 2] / cell));
      let arr = grid.get(kk);
      if (!arr) grid.set(kk, (arr = []));
      arr.push(i);
    }
    const ds = [];
    for (let s = 0; s < 600; s++) {
      const i = Math.floor(Math.floor((s * 7919) % m) * step) * 3;
      const cx = Math.floor(pos[i] / cell), cy = Math.floor(pos[i + 1] / cell), cz = Math.floor(pos[i + 2] / cell);
      let best = Infinity;
      for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) for (let c = -1; c <= 1; c++) {
        for (const q of grid.get(key(cx + a, cy + b, cz + c)) || []) {
          if (q === i) continue;
          const d = (pos[q] - pos[i]) ** 2 + (pos[q + 1] - pos[i + 1]) ** 2 + (pos[q + 2] - pos[i + 2]) ** 2;
          if (d > 0 && d < best) best = d;
        }
      }
      if (best < Infinity) ds.push(Math.sqrt(best));
    }
    if (ds.length > 300) {
      ds.sort((a, b) => a - b);
      // the sample is sparser than the full cloud by sqrt(count / m) on a surface
      return ds[ds.length >> 1] / Math.sqrt(count / m);
    }
    cell *= 2.5;
  }
  return diag / Math.sqrt(count);
}

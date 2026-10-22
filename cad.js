// DWG / DXF -> raster canvas at a known pixels-per-meter, so CAD drawings arrive already to scale.
// DXF is parsed with dxf-parser, DWG with libredwg (WebAssembly). Both are normalised into the
// same entity walk below.

// window.__LIBS lets a self-hosted build point at local copies
const DXF_PARSER = window.__LIBS?.dxf ?? "https://cdn.jsdelivr.net/npm/dxf-parser@1.1.2/dist/dxf-parser.js"; // UMD build -> self.DxfParser
const LIBREDWG = window.__LIBS?.dwg ?? "https://cdn.jsdelivr.net/npm/@mlightcad/libredwg-web@0.7.0/dist/libredwg-web.js";

// $INSUNITS -> metres
const UNITS = { 1: 0.0254, 2: 0.3048, 4: 0.001, 5: 0.01, 6: 1, 14: 0.1 };

let dxfParserMod, libredwg;

function loadScript(src) {
  return new Promise((resolve, reject) => {
    const el = document.createElement("script");
    el.src = src;
    el.onload = resolve;
    el.onerror = () => reject(new Error(`${src} 로드 실패`));
    document.head.appendChild(el);
  });
}

async function readDxf(file) {
  if (!dxfParserMod) {
    await loadScript(DXF_PARSER);
    const g = self.DxfParser;
    dxfParserMod = g.default || g.DxfParser || g;
  }
  const dxf = new dxfParserMod().parseSync(await file.text());
  const blocks = {};
  for (const [name, b] of Object.entries(dxf.blocks || {})) blocks[name] = { base: b.position || { x: 0, y: 0 }, entities: b.entities || [] };
  return { entities: dxf.entities || [], blocks, insUnits: dxf.header?.$INSUNITS, angDeg: true };
}

async function readDwg(file) {
  if (!libredwg) {
    const m = await import(LIBREDWG);
    libredwg = { lib: await m.LibreDwg.create(), type: m.Dwg_File_Type.DWG };
  }
  const { lib, type } = libredwg;
  const data = lib.dwg_read_data(await file.arrayBuffer(), type);
  if (data == null) throw new Error("DWG 파일을 해석하지 못했습니다 (지원 버전: R14 ~ 2018 계열)");
  try {
    const db = lib.convert(data);
    const blocks = {};
    let model = null;
    for (const br of db.tables?.BLOCK_RECORD?.entries || []) {
      blocks[br.name] = { base: br.basePoint || { x: 0, y: 0 }, entities: br.entities || [] };
      if (br.name.toLowerCase() === "*model_space") model = br.entities;
    }
    return { entities: model || db.entities || [], blocks, insUnits: db.header?.INSUNITS, angDeg: false };
  } finally {
    lib.dwg_free(data);
  }
}

// 2D affine [a, b, c, d, e, f]: x' = a x + c y + e, y' = b x + d y + f
const mul = (m, n) => [
  m[0] * n[0] + m[2] * n[1], m[1] * n[0] + m[3] * n[1],
  m[0] * n[2] + m[2] * n[3], m[1] * n[2] + m[3] * n[3],
  m[0] * n[4] + m[2] * n[5] + m[4], m[1] * n[4] + m[3] * n[5] + m[5],
];
const apply = (m, x, y) => [m[0] * x + m[2] * y + m[4], m[1] * x + m[3] * y + m[5]];

function collect(src) {
  const segs = []; // flat [x1, y1, x2, y2, ...]
  const texts = [];
  const rot = (v) => (src.angDeg ? (v || 0) * Math.PI / 180 : v || 0);
  const line = (m, pts, closed) => {
    for (let i = 0; i + 1 < pts.length; i++) segs.push(...apply(m, pts[i][0], pts[i][1]), ...apply(m, pts[i + 1][0], pts[i + 1][1]));
    if (closed && pts.length > 2) segs.push(...apply(m, pts.at(-1)[0], pts.at(-1)[1]), ...apply(m, pts[0][0], pts[0][1]));
  };
  const arc = (cx, cy, r, a0, a1) => {
    while (a1 <= a0) a1 += Math.PI * 2;
    const n = Math.max(8, Math.ceil((a1 - a0) / (Math.PI / 36)));
    return Array.from({ length: n + 1 }, (_, i) => { const a = a0 + (a1 - a0) * i / n; return [cx + r * Math.cos(a), cy + r * Math.sin(a)]; });
  };
  const bulgePts = (vs, closed) => {
    const out = [];
    const n = vs.length;
    for (let i = 0; i < n; i++) {
      const a = vs[i];
      out.push([a.x, a.y]);
      if (!a.bulge || (i === n - 1 && !closed)) continue;
      const b = vs[(i + 1) % n];
      const th = 4 * Math.atan(a.bulge), dx = b.x - a.x, dy = b.y - a.y, c = Math.hypot(dx, dy);
      if (c < 1e-12) continue;
      const r = c / (2 * Math.sin(th / 2));
      const h = r * Math.cos(th / 2);
      const cx = (a.x + b.x) / 2 - h * dy / c, cy = (a.y + b.y) / 2 + h * dx / c;
      const a0 = Math.atan2(a.y - cy, a.x - cx), rr = Math.abs(r);
      for (let k = 1; k < 12; k++) { const t = a0 + th * k / 12; out.push([cx + rr * Math.cos(t), cy + rr * Math.sin(t)]); }
    }
    return out;
  };
  const walk = (entities, m, depth) => {
    for (const e of entities) {
      if (e.isVisible === false || e.visible === false) continue;
      switch (e.type) {
        case "LINE": {
          const a = e.startPoint || e.vertices?.[0], b = e.endPoint || e.vertices?.[1];
          if (a && b) line(m, [[a.x, a.y], [b.x, b.y]]);
          break;
        }
        case "LWPOLYLINE": case "POLYLINE": case "POLYLINE2D": case "POLYLINE3D": {
          const vs = (e.vertices || []).filter((v) => v && Number.isFinite(v.x));
          const closed = !!(e.shape || e.closed || (e.flag ?? e.flags ?? 0) & 1 || (e.flag ?? 0) & 512);
          line(m, bulgePts(vs, closed), closed);
          break;
        }
        case "CIRCLE": line(m, arc(e.center.x, e.center.y, e.radius, 0, Math.PI * 2)); break;
        case "ARC": line(m, arc(e.center.x, e.center.y, e.radius, e.startAngle, e.endAngle)); break;
        case "ELLIPSE": {
          const mx = e.majorAxisEndPoint.x, my = e.majorAxisEndPoint.y, ratio = e.axisRatio;
          let a0 = e.startAngle ?? 0, a1 = e.endAngle ?? Math.PI * 2;
          while (a1 <= a0) a1 += Math.PI * 2;
          const n = 64, pts = [];
          for (let i = 0; i <= n; i++) {
            const t = a0 + (a1 - a0) * i / n, c = Math.cos(t), s = Math.sin(t) * ratio;
            pts.push([e.center.x + mx * c - my * s, e.center.y + my * c + mx * s]);
          }
          line(m, pts);
          break;
        }
        case "SPLINE": {
          const pts = (e.fitPoints?.length ? e.fitPoints : e.controlPoints) || [];
          line(m, pts.map((p) => [p.x, p.y]));
          break;
        }
        case "SOLID": case "3DFACE": {
          const cs = [e.corner1, e.corner2, e.corner4 || e.corner3, e.corner3].filter(Boolean);
          const vs = e.points || cs;
          if (vs.length > 2) line(m, vs.map((p) => [p.x, p.y]), true);
          break;
        }
        case "TEXT": case "MTEXT": case "ATTRIB": {
          const p = e.startPoint || e.insertionPoint || e.position;
          const h = e.textHeight || e.height || e.nominalTextHeight;
          let t = e.text ?? e.contents ?? "";
          if (!p || !h || !t) break;
          t = String(t).replace(/\\P/g, " ").replace(/\\[A-Za-z][^;\\]*;/g, "").replace(/[{}]/g, "").replace(/\\[A-Za-z]/g, "");
          const [x, y] = apply(m, p.x, p.y);
          texts.push({ x, y, h: h * Math.sqrt(Math.abs(m[0] * m[3] - m[1] * m[2])), r: rot(e.rotation) + Math.atan2(m[1], m[0]), t });
          break;
        }
        case "INSERT": case "DIMENSION": {
          const blk = src.blocks[e.name || e.block];
          if (!blk || depth > 10) break;
          const p = e.insertionPoint || e.position || { x: 0, y: 0 };
          const sx = e.xScale ?? 1, sy = e.yScale ?? 1;
          const r = e.type === "INSERT" ? rot(e.rotation) : 0;
          const c = Math.cos(r), s = Math.sin(r);
          const local = e.type === "INSERT"
            ? mul([c * sx, s * sx, -s * sy, c * sy, p.x, p.y], [1, 0, 0, 1, -blk.base.x, -blk.base.y])
            : [1, 0, 0, 1, 0, 0]; // dimension blocks are stored in world coordinates
          walk(blk.entities, mul(m, local), depth + 1);
          if (e.attribs) walk(e.attribs, m, depth + 1);
          break;
        }
      }
    }
  };
  walk(src.entities, [1, 0, 0, 1, 0, 0], 0);
  return { segs, texts };
}

// bounding box that ignores a few stray far-away entities (common in real drawings)
function robustExtents(segs) {
  const xs = [], ys = [];
  for (let i = 0; i < segs.length; i += 2) { xs.push(segs[i]); ys.push(segs[i + 1]); }
  const full = [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
  const sx = [...xs].sort((a, b) => a - b), sy = [...ys].sort((a, b) => a - b);
  const q = (arr, f) => arr[Math.min(arr.length - 1, Math.floor(f * arr.length))];
  const core = [q(sx, 0.005), q(sy, 0.005), q(sx, 0.995), q(sy, 0.995)];
  const span = Math.max(core[2] - core[0], core[3] - core[1]);
  const fullSpan = Math.max(full[2] - full[0], full[3] - full[1]);
  if (fullSpan <= span * 3) return full;
  const pad = span * 0.15;
  return [core[0] - pad, core[1] - pad, core[2] + pad, core[3] + pad];
}

/**
 * @param {File} file
 * @param {{maxTex:number, askUnit:(msg:string)=>Promise<number|null>}} opts
 * @returns {Promise<{canvas:HTMLCanvasElement, ppm:number, info:string}>}
 */
export async function loadCad(file, { maxTex, askUnit }) {
  const ext = file.name.split(".").pop().toLowerCase();
  const src = ext === "dwg" ? await readDwg(file) : await readDxf(file);
  let unit = UNITS[src.insUnits];
  if (!unit) {
    const mm = await askUnit("도면 단위 정보가 없습니다. 1 도면단위가 몇 mm인지 입력하세요 (건축 도면은 보통 1 = 1 mm).");
    if (!mm) return null;
    unit = mm / 1000;
  }
  const { segs, texts } = collect(src);
  if (!segs.length) throw new Error("그릴 수 있는 선 요소가 없습니다 (LINE / POLYLINE / ARC / CIRCLE / INSERT 등)");
  let [minX, minY, maxX, maxY] = robustExtents(segs);
  const pad = 0.02 * Math.max(maxX - minX, maxY - minY);
  minX -= pad; minY -= pad; maxX += pad; maxY += pad;
  const wM = (maxX - minX) * unit, hM = (maxY - minY) * unit;
  const ppm = Math.min(400, maxTex / Math.max(wM, hM)); // never finer than 2.5 mm per pixel
  const cv = document.createElement("canvas");
  cv.width = Math.max(1, Math.ceil(wM * ppm));
  cv.height = Math.max(1, Math.ceil(hM * ppm));
  const ctx = cv.getContext("2d");
  ctx.fillStyle = "#fff";
  ctx.fillRect(0, 0, cv.width, cv.height);
  const k = unit * ppm;
  ctx.strokeStyle = "#000";
  ctx.lineWidth = Math.max(1.25, ppm * 0.008);
  ctx.lineCap = "round";
  ctx.beginPath();
  for (let i = 0; i < segs.length; i += 4) {
    ctx.moveTo((segs[i] - minX) * k, (maxY - segs[i + 1]) * k);
    ctx.lineTo((segs[i + 2] - minX) * k, (maxY - segs[i + 3]) * k);
  }
  ctx.stroke();
  ctx.fillStyle = "#000";
  ctx.textBaseline = "alphabetic";
  for (const t of texts) {
    const px = t.h * k;
    if (px < 4) continue;
    ctx.save();
    ctx.translate((t.x - minX) * k, (maxY - t.y) * k);
    ctx.rotate(-t.r);
    ctx.font = `${px}px Pretendard, sans-serif`;
    ctx.fillText(t.t, 0, 0);
    ctx.restore();
  }
  // segment endpoints in canvas pixels, for exact snapping when picking points
  const seen = new Set(), snap = [];
  for (let i = 0; i < segs.length && snap.length < 600_000; i += 2) {
    const px = (segs[i] - minX) * k, py = (maxY - segs[i + 1]) * k;
    if (px < 0 || py < 0 || px > cv.width || py > cv.height) continue;
    const key = `${Math.round(px * 2)},${Math.round(py * 2)}`;
    if (seen.has(key)) continue;
    seen.add(key);
    snap.push(px, py);
  }
  return {
    canvas: cv, ppm, snap: Float32Array.from(snap),
    info: `${ext.toUpperCase()} 선분 ${(segs.length / 4).toLocaleString()}개 · 단위 ${+(unit * 1000).toFixed(3)} mm · ${wM.toFixed(1)}×${hM.toFixed(1)} m — 실척 자동 배치`,
  };
}

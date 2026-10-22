// Point snapping for the 2D drawing picker.
//  - CAD drawings (DWG/DXF): exact segment endpoints, looked up through a bucket grid
//  - raster drawings (PDF / images): the strongest line corner near the cursor (Shi-Tomasi)

const BUCKET = 32; // px

export function snapIndex(pts) {
  const map = new Map();
  for (let i = 0; i < pts.length; i += 2) {
    const key = `${Math.floor(pts[i] / BUCKET)},${Math.floor(pts[i + 1] / BUCKET)}`;
    let arr = map.get(key);
    if (!arr) map.set(key, (arr = []));
    arr.push(i);
  }
  return { pts, map };
}

/** nearest CAD endpoint within `radius` image px of (x, y), or null */
export function snapEndpoint(index, x, y, radius) {
  const { pts, map } = index;
  const r = Math.ceil(radius / BUCKET);
  const bx = Math.floor(x / BUCKET), by = Math.floor(y / BUCKET);
  let best = null, bd = radius * radius;
  for (let i = -r; i <= r; i++) for (let j = -r; j <= r; j++) {
    for (const k of map.get(`${bx + i},${by + j}`) || []) {
      const d = (pts[k] - x) ** 2 + (pts[k + 1] - y) ** 2;
      if (d <= bd) { bd = d; best = { x: pts[k], y: pts[k + 1], kind: "끝점" }; }
    }
  }
  return best;
}

/**
 * strongest corner of dark linework within `radius` image px, or null.
 * Shi-Tomasi: the smaller eigenvalue of the gradient structure tensor over a 5x5 window.
 */
export function snapCorner(canvas, x, y, radius) {
  const R = Math.max(3, Math.min(60, Math.round(radius)));
  const x0 = Math.max(0, Math.round(x) - R - 3), y0 = Math.max(0, Math.round(y) - R - 3);
  const w = Math.min(canvas.width - x0, 2 * R + 7), h = Math.min(canvas.height - y0, 2 * R + 7);
  if (w < 9 || h < 9) return null;
  const data = canvas.getContext("2d", { willReadFrequently: true }).getImageData(x0, y0, w, h).data;
  const g = new Float32Array(w * h);
  let dark = 0;
  for (let i = 0; i < w * h; i++) {
    g[i] = (data[i * 4] * 0.299 + data[i * 4 + 1] * 0.587 + data[i * 4 + 2] * 0.114) / 255;
    if (g[i] < 0.55) dark++;
  }
  if (!dark) return null;
  const ix = new Float32Array(w * h), iy = new Float32Array(w * h);
  for (let yy = 1; yy < h - 1; yy++) for (let xx = 1; xx < w - 1; xx++) {
    const i = yy * w + xx;
    ix[i] = (g[i - w + 1] + 2 * g[i + 1] + g[i + w + 1] - g[i - w - 1] - 2 * g[i - 1] - g[i + w - 1]) / 4;
    iy[i] = (g[i + w - 1] + 2 * g[i + w] + g[i + w + 1] - g[i - w - 1] - 2 * g[i - w] - g[i - w + 1]) / 4;
  }
  let best = null, bestScore = 0.6; // minimum corner strength, in gradient units over the window
  const cx = x - x0, cy = y - y0;
  for (let yy = 3; yy < h - 3; yy++) for (let xx = 3; xx < w - 3; xx++) {
    if ((xx - cx) ** 2 + (yy - cy) ** 2 > R * R) continue;
    let a = 0, b = 0, c = 0;
    for (let dy = -2; dy <= 2; dy++) for (let dx = -2; dx <= 2; dx++) {
      const i = (yy + dy) * w + xx + dx;
      a += ix[i] * ix[i]; b += ix[i] * iy[i]; c += iy[i] * iy[i];
    }
    const lmin = (a + c) / 2 - Math.sqrt(((a - c) / 2) ** 2 + b * b);
    // prefer strong corners, then closeness to the cursor
    const score = lmin * (1 - 0.3 * Math.hypot(xx - cx, yy - cy) / R);
    if (score > bestScore) { bestScore = score; best = { x: x0 + xx, y: y0 + yy, kind: "코너" }; }
  }
  return best;
}

// Scan -> IFC: walls from a wall-height slice of the scan, a floor slab from the detected floor,
// and the scan's colour markers, written as a fresh IFC4 model in the scan's own coordinates.
// Viewer coordinates are Y-up; IFC is Z-up: IFC (X, Y, Z) = viewer (x, -z, y).
import { ifcGuid, real, str } from "./ifcexport.js";

// ---------------------------------------------------------------- wall faces from the slice
function fitLine(S, idx) {
  let cx = 0, cz = 0;
  for (const i of idx) { cx += S[i]; cz += S[i + 1]; }
  cx /= idx.length; cz /= idx.length;
  let xx = 0, xz = 0, zz = 0;
  for (const i of idx) { const dx = S[i] - cx, dz = S[i + 1] - cz; xx += dx * dx; xz += dx * dz; zz += dz * dz; }
  const ang = 0.5 * Math.atan2(2 * xz, xx - zz); // principal direction
  return { cx, cz, dx: Math.cos(ang), dz: Math.sin(ang) };
}

/**
 * Sequential RANSAC over 2D slice points [x, z, ...]: each line is split at gaps into wall faces.
 * tol / gap / minLen are in metres (multiplied by `unit` for non-metric scans).
 */
export function extractFaces(S, { unit = 1, tol = 0.03, gap = 0.4, minLen = 0.6, density = 0.25, maxLines = 40 } = {}) {
  tol *= unit; gap *= unit; minLen *= unit;
  const n = S.length / 2;
  let alive = Array.from({ length: n }, (_, k) => k * 2);
  const faces = [];
  let seed = 7;
  const rnd = () => ((seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
  for (let line = 0; line < maxLines && alive.length > 40; line++) {
    const sub = alive.length > 2500 ? Array.from({ length: 2500 }, () => alive[Math.floor(rnd() * alive.length)]) : alive;
    let best = null, bestCount = 0;
    for (let it = 0; it < 300; it++) {
      const a = alive[Math.floor(rnd() * alive.length)], b = alive[Math.floor(rnd() * alive.length)];
      let dx = S[b] - S[a], dz = S[b + 1] - S[a + 1];
      const l = Math.hypot(dx, dz);
      if (l < minLen * 0.5) continue;
      dx /= l; dz /= l;
      let c = 0;
      for (const i of sub) if (Math.abs((S[i] - S[a]) * dz - (S[i + 1] - S[a + 1]) * dx) < tol) c++;
      if (c > bestCount) { bestCount = c; best = { cx: S[a], cz: S[a + 1], dx, dz }; }
    }
    if (!best) break;
    let L = best, inl = [];
    for (let r = 0; r < 2; r++) {
      inl = alive.filter((i) => Math.abs((S[i] - L.cx) * L.dz - (S[i + 1] - L.cz) * L.dx) < tol);
      if (inl.length < 10) break;
      L = fitLine(S, inl);
    }
    if (inl.length < 30) break;
    // split along the line where the points have gaps
    const ts = inl.map((i) => ({ i, t: (S[i] - L.cx) * L.dx + (S[i + 1] - L.cz) * L.dz })).sort((p, q) => p.t - q.t);
    let start = 0;
    for (let k = 1; k <= ts.length; k++) {
      if (k === ts.length || ts[k].t - ts[k - 1].t > gap) {
        const t0 = ts[start].t, t1 = ts[k - 1].t, cnt = k - start;
        // keep runs that are long and reasonably dense (rejects furniture edges and stray splats)
        if (t1 - t0 >= minLen && cnt >= (t1 - t0) / (0.02 * unit) * density) {
          faces.push({ cx: L.cx, cz: L.cz, dx: L.dx, dz: L.dz, t0, t1, count: cnt });
        }
        start = k;
      }
    }
    const used = new Set(inl);
    alive = alive.filter((i) => !used.has(i));
  }
  return faces;
}

// ---------------------------------------------------------------- faces -> walls (centre lines)
const angOf = (f) => Math.atan2(f.dz, f.dx);
const wrapPi = (a) => { a %= Math.PI; return a < 0 ? a + Math.PI : a; };

/**
 * @param faces    from extractFaces
 * @param interior a point inside the room (x, z), used to put single faces' thickness outside
 * conservative: as few masses as possible — strong orthogonal snapping, off-grid faces dropped unless
 *   long, collinear pieces bridged across openings, duplicates and short stubs removed.
 */
export function facesToWalls(faces, interior, { unit = 1, thickness = 0.2, orthogonal = true, conservative = true } = {}) {
  const stats = { faces: faces.length, droppedOffGrid: 0, mergedFaces: 0, duplicates: 0, short: 0 };
  // dominant direction (mod 90 deg), length weighted
  let sx = 0, sy = 0;
  for (const f of faces) { const a = angOf(f) * 4, w = f.t1 - f.t0; sx += Math.cos(a) * w; sy += Math.sin(a) * w; }
  const dom = Math.atan2(sy, sx) / 4;
  const snapTol = (conservative ? 15 : 6) * Math.PI / 180;
  const mid = (f) => { const tm = (f.t0 + f.t1) / 2; return [f.cx + f.dx * tm, f.cz + f.dz * tm]; };
  // re-anchor a face at its midpoint with direction `ang`
  const reorient = (f, ang) => {
    const [mx, mz] = mid(f), half = (f.t1 - f.t0) / 2;
    return { ...f, cx: mx, cz: mz, dx: Math.cos(ang), dz: Math.sin(ang), t0: -half, t1: half };
  };
  let F = [];
  for (const f of faces) {
    if (!orthogonal) { F.push(f); continue; }
    const a = angOf(f), k = Math.round((a - dom) / (Math.PI / 2)), target = dom + k * Math.PI / 2;
    if (Math.abs(a - target) <= snapTol) F.push(reorient(f, target));
    else if (!conservative || f.t1 - f.t0 >= 3 * unit) F.push(f);
    else stats.droppedOffGrid++;
  }

  const parallel = (f, g, tol) => Math.abs(wrapPi(angOf(f) - angOf(g) + Math.PI / 2) - Math.PI / 2) <= tol;
  const proj = (f, x, z) => (x - f.cx) * f.dx + (z - f.cz) * f.dz;
  const off = (f, x, z) => (x - f.cx) * f.dz - (z - f.cz) * f.dx; // signed distance from f's line
  const endpoints = (f) => [[f.cx + f.dx * f.t0, f.cz + f.dz * f.t0], [f.cx + f.dx * f.t1, f.cz + f.dz * f.t1]];

  // collinear pieces of one face (split by doors or furniture) become one face
  const bridge = (conservative ? 1.2 : 0.4) * unit;
  if (conservative) {
    let merged = true;
    while (merged) {
      merged = false;
      outer: for (let i = 0; i < F.length; i++) for (let j = i + 1; j < F.length; j++) {
        const f = F[i], g = F[j];
        if (!parallel(f, g, 3 * Math.PI / 180)) continue;
        const ge = endpoints(g).map(([x, z]) => [proj(f, x, z), off(f, x, z)]);
        if (Math.abs((ge[0][1] + ge[1][1]) / 2) > 0.06 * unit) continue;
        const g0 = Math.min(ge[0][0], ge[1][0]), g1 = Math.max(ge[0][0], ge[1][0]);
        const gapLen = Math.max(g0 - f.t1, f.t0 - g1);
        if (gapLen > bridge) continue;
        // length-weighted average offset keeps the merged face on the stronger piece
        const lf = f.t1 - f.t0, lg = g1 - g0, o = ((ge[0][1] + ge[1][1]) / 2) * lg / (lf + lg);
        F[i] = { ...f, cx: f.cx + f.dz * o, cz: f.cz - f.dx * o, t0: Math.min(f.t0, g0), t1: Math.max(f.t1, g1), count: f.count + g.count };
        F.splice(j, 1);
        stats.mergedFaces++;
        merged = true;
        break outer;
      }
    }
  }

  // pair parallel faces 6-45 cm apart that overlap: the two sides of one wall
  F = F.map((f) => ({ ...f, used: false }));
  let walls = [];
  for (let i = 0; i < F.length; i++) {
    if (F[i].used) continue;
    let best = null;
    for (let j = 0; j < F.length; j++) {
      if (j === i || F[j].used || !parallel(F[i], F[j], 4 * Math.PI / 180)) continue;
      const mj = endpoints(F[j]).map(([x, z]) => [proj(F[i], x, z), off(F[i], x, z)]);
      const d = Math.abs((mj[0][1] + mj[1][1]) / 2);
      if (d < 0.06 * unit || d > 0.45 * unit) continue;
      const lo = Math.max(F[i].t0, Math.min(mj[0][0], mj[1][0])), hi = Math.min(F[i].t1, Math.max(mj[0][0], mj[1][0]));
      const overlap = hi - lo, shorter = Math.min(F[i].t1 - F[i].t0, Math.abs(mj[1][0] - mj[0][0]));
      if (overlap < 0.5 * shorter) continue;
      if (!best || d < best.d) best = { j, d, mj };
    }
    const f = F[i];
    if (best) {
      F[best.j].used = f.used = true;
      const sOff = (best.mj[0][1] + best.mj[1][1]) / 2;
      const t0 = Math.min(f.t0, best.mj[0][0], best.mj[1][0]), t1 = Math.max(f.t1, best.mj[0][0], best.mj[1][0]);
      const ox = f.dz * sOff / 2, oz = -f.dx * sOff / 2; // centre line halfway between the faces
      walls.push({ a: [f.cx + ox + f.dx * t0, f.cz + oz + f.dz * t0], b: [f.cx + ox + f.dx * t1, f.cz + oz + f.dz * t1], t: best.d, paired: true });
    } else {
      f.used = true;
      const side = off(f, interior[0], interior[1]) > 0 ? -1 : 1; // the wall body lies away from the room
      const h = (thickness * unit) / 2 * side, ox = f.dz * h, oz = -f.dx * h;
      walls.push({ a: [f.cx + ox + f.dx * f.t0, f.cz + oz + f.dz * f.t0], b: [f.cx + ox + f.dx * f.t1, f.cz + oz + f.dz * f.t1], t: thickness * unit, paired: false });
    }
  }

  if (conservative) {
    const len = (w) => Math.hypot(w.b[0] - w.a[0], w.b[1] - w.a[1]);
    const asFace = (w) => { const L = len(w) || 1, dx = (w.b[0] - w.a[0]) / L, dz = (w.b[1] - w.a[1]) / L; return { cx: w.a[0], cz: w.a[1], dx, dz, t0: 0, t1: L }; };
    // a wall lying inside a longer parallel wall's thickness is the same mass twice
    walls.sort((p, q) => len(q) - len(p));
    const kept = [];
    for (const w of walls) {
      const dup = kept.some((v) => {
        const fv = asFace(v);
        if (!parallel(fv, asFace(w), 6 * Math.PI / 180)) return false;
        const e = [w.a, w.b].map(([x, z]) => [proj(fv, x, z), off(fv, x, z)]);
        if (Math.abs((e[0][1] + e[1][1]) / 2) > Math.max(v.t, w.t)) return false;
        const lo = Math.max(0, Math.min(e[0][0], e[1][0])), hi = Math.min(fv.t1, Math.max(e[0][0], e[1][0]));
        return hi - lo >= 0.6 * len(w);
      });
      if (dup) stats.duplicates++;
      else kept.push(w);
    }
    walls = kept.filter((w) => {
      const ok = len(w) >= 1.0 * unit;
      if (!ok) stats.short++;
      return ok;
    });
  }
  joinCorners(walls, 0.6 * unit);
  walls.stats = stats;
  return walls;
}

// extend / trim wall ends that stop near another wall's centre line so corners close
function joinCorners(walls, reach) {
  for (const w of walls) {
    for (const end of ["a", "b"]) {
      const p = w[end], q = w[end === "a" ? "b" : "a"];
      const d1 = [p[0] - q[0], p[1] - q[1]];
      let best = null;
      for (const v of walls) {
        if (v === w) continue;
        const d2 = [v.b[0] - v.a[0], v.b[1] - v.a[1]];
        const den = d1[0] * d2[1] - d1[1] * d2[0];
        const l1 = Math.hypot(...d1), l2 = Math.hypot(...d2);
        if (Math.abs(den) < 0.5 * l1 * l2) continue; // nearly parallel (< 30 deg)
        const s = ((v.a[0] - q[0]) * d2[1] - (v.a[1] - q[1]) * d2[0]) / den; // along w from q
        const u = ((v.a[0] - q[0]) * d1[1] - (v.a[1] - q[1]) * d1[0]) / den; // along v
        const x = q[0] + d1[0] * s, z = q[1] + d1[1] * s;
        const move = Math.hypot(x - p[0], z - p[1]);
        const onV = u >= -reach / l2 && u <= 1 + reach / l2;
        if (move <= reach && onV && (!best || move < best.move)) best = { x, z, move };
      }
      if (best) { p[0] = best.x; p[1] = best.z; }
    }
  }
}

// ---------------------------------------------------------------- IFC4 writer
/**
 * Shared IFC4 skeleton: project (metres) / site / building / one storey, surface styles, red marker
 * cubes, and the containment relation. Callers add their own geometry through the helpers.
 */
function ifcWriter(name, description, storeyElevation = 0) {
  const lines = [];
  let n = 0;
  const add = (body) => { lines.push(`#${++n}= ${body};`); return n; };
  const P3 = (x, y, z) => add(`IFCCARTESIANPOINT((${real(x)},${real(y)},${real(z)}))`);
  const origin = P3(0, 0, 0);
  const zDir = add("IFCDIRECTION((0.,0.,1.))"), xDir = add("IFCDIRECTION((1.,0.,0.))");
  const wcs = add(`IFCAXIS2PLACEMENT3D(#${origin},#${zDir},#${xDir})`);
  const ctx = add(`IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,1.E-05,#${wcs},$)`);
  const body = add(`IFCGEOMETRICREPRESENTATIONSUBCONTEXT('Body','Model',*,*,*,*,#${ctx},$,.MODEL_VIEW.,$)`);
  const uLen = add("IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.)");
  const uArea = add("IFCSIUNIT(*,.AREAUNIT.,$,.SQUARE_METRE.)");
  const uVol = add("IFCSIUNIT(*,.VOLUMEUNIT.,$,.CUBIC_METRE.)");
  const uAng = add("IFCSIUNIT(*,.PLANEANGLEUNIT.,$,.RADIAN.)");
  const units = add(`IFCUNITASSIGNMENT((#${uLen},#${uArea},#${uVol},#${uAng}))`);
  const project = add(`IFCPROJECT('${ifcGuid()}',$,${str(name)},${str(description)},$,$,$,(#${ctx}),#${units})`);
  const place = () => add(`IFCLOCALPLACEMENT($,#${add(`IFCAXIS2PLACEMENT3D(#${P3(0, 0, 0)},$,$)`)})`);
  const site = add(`IFCSITE('${ifcGuid()}',$,'Site',$,$,#${place()},$,$,.ELEMENT.,$,$,$,$,$)`);
  const building = add(`IFCBUILDING('${ifcGuid()}',$,'Building',$,$,#${place()},$,$,.ELEMENT.,$,$,$)`);
  const storey = add(`IFCBUILDINGSTOREY('${ifcGuid()}',$,'Level 1',$,$,#${place()},$,$,.ELEMENT.,${real(storeyElevation)})`);
  add(`IFCRELAGGREGATES('${ifcGuid()}',$,$,$,#${project},(#${site}))`);
  add(`IFCRELAGGREGATES('${ifcGuid()}',$,$,$,#${site},(#${building}))`);
  add(`IFCRELAGGREGATES('${ifcGuid()}',$,$,$,#${building},(#${storey}))`);

  const style = (r, g, b, label) => {
    const c = add(`IFCCOLOURRGB(${str(label)},${real(r)},${real(g)},${real(b)})`);
    const rd = add(`IFCSURFACESTYLERENDERING(#${c},0.,$,$,$,$,$,$,.NOTDEFINED.)`);
    return add(`IFCSURFACESTYLE(${str(label)},.BOTH.,(#${rd}))`);
  };
  const shape = (item, st, repType = "SweptSolid") => {
    add(`IFCSTYLEDITEM(#${item},(#${st}),$)`);
    const rep = add(`IFCSHAPEREPRESENTATION(#${body},'Body',${str(repType)},(#${item}))`);
    return add(`IFCPRODUCTDEFINITIONSHAPE($,$,(#${rep}))`);
  };
  const contained = [];
  let markerStyle = null;
  // 20 cm red cubes centred on each marker point (IFC coordinates)
  const addMarkers = (markers) => markers.forEach((mk) => {
    markerStyle ??= style(1, 0, 0, "Marker");
    const lp = add(`IFCLOCALPLACEMENT($,#${add(`IFCAXIS2PLACEMENT3D(#${P3(...mk.ifc)},$,$)`)})`);
    const p2 = add(`IFCAXIS2PLACEMENT2D(#${add("IFCCARTESIANPOINT((0.,0.))")},$)`);
    const prof = add(`IFCRECTANGLEPROFILEDEF(.AREA.,'Marker',#${p2},0.2,0.2)`);
    const solid = add(`IFCEXTRUDEDAREASOLID(#${prof},#${add(`IFCAXIS2PLACEMENT3D(#${P3(0, 0, -0.1)},$,$)`)},#${zDir},0.2)`);
    contained.push(add(`IFCBUILDINGELEMENTPROXY('${ifcGuid()}',$,${str(mk.name)},${str(mk.desc || "Marker")},'Marker',#${lp},#${shape(solid, markerStyle)},${str(mk.tag || "")},.NOTDEFINED.)`));
  });
  const finish = () => {
    if (contained.length) add(`IFCRELCONTAINEDINSPATIALSTRUCTURE('${ifcGuid()}',$,'Level 1 elements',$,(${contained.map((c) => "#" + c).join(",")}),#${storey})`);
    const stamp = new Date().toISOString().slice(0, 19);
    return [
      "ISO-10303-21;", "HEADER;",
      "FILE_DESCRIPTION(('ViewDefinition [ReferenceView]'),'2;1');",
      `FILE_NAME(${str(name)},'${stamp}',(''),(''),'zzikbang','zzikbang','');`,
      "FILE_SCHEMA(('IFC4'));", "ENDSEC;", "DATA;", ...lines, "ENDSEC;", "END-ISO-10303-21;", "",
    ].join("\n");
  };
  return { add, P3, zDir, style, shape, place, contained, addMarkers, finish };
}

// viewer (Y-up) -> IFC (Z-up)
const viewerToIfc = (v) => [v.x, -v.z, v.y];

/**
 * @param walls   [{a:[x,z], b:[x,z], t, y0?, h?, kind?, label?}] centre lines in viewer coordinates;
 *                y0/h override the base and height, kind "proxy" writes a generic object instead of a wall
 * @param floorY  viewer height of the floor; height = wall height (same units)
 * @param slab    polygon [[x,z], ...] for the floor slab (viewer coordinates)
 * @param markers [{name, tag, pos: Vector3 (viewer)}]
 */
export function buildScanIfc({ name, walls, floorY, height, slab, markers }) {
  const W = ifcWriter(name, "Generated from a 3DGS scan", floorY);
  const { add, P3, zDir, style, shape, place, contained } = W;
  const toIfc = (x, z, y = floorY) => [x, -z, y];
  const wallStyle = style(0.86, 0.85, 0.82, "Wall"), slabStyle = style(0.62, 0.62, 0.64, "Slab");

  // walls: rectangle profile along the centre line, extruded up from the floor
  walls.forEach((w, i) => {
    const [ax, ay, az] = toIfc(w.a[0], w.a[1], w.y0 ?? floorY), [bx, by] = toIfc(w.b[0], w.b[1]);
    const L = Math.hypot(bx - ax, by - ay);
    if (L < 1e-3) return;
    const dir = add(`IFCDIRECTION((${real((bx - ax) / L)},${real((by - ay) / L)},0.))`);
    const lp = add(`IFCLOCALPLACEMENT($,#${add(`IFCAXIS2PLACEMENT3D(#${P3(ax, ay, az)},#${zDir},#${dir})`)})`);
    const p2 = add(`IFCAXIS2PLACEMENT2D(#${add(`IFCCARTESIANPOINT((${real(L / 2)},0.))`)},$)`);
    const prof = add(`IFCRECTANGLEPROFILEDEF(.AREA.,$,#${p2},${real(L)},${real(w.t)})`);
    const solid = add(`IFCEXTRUDEDAREASOLID(#${prof},#${add(`IFCAXIS2PLACEMENT3D(#${P3(0, 0, 0)},$,$)`)},#${zDir},${real(w.h ?? height)})`);
    const label = w.label || `Wall-${String(i + 1).padStart(2, "0")}`;
    if (w.kind === "proxy") contained.push(add(`IFCBUILDINGELEMENTPROXY('${ifcGuid()}',$,${str(label)},'voxel block from scan','Object',#${lp},#${shape(solid, wallStyle)},$,.NOTDEFINED.)`));
    else contained.push(add(`IFCWALL('${ifcGuid()}',$,${str(label)},${str(w.note || (w.paired ? "both faces scanned" : "one face scanned"))},$,#${lp},#${shape(solid, wallStyle)},$,.STANDARD.)`));
  });

  // floor slab: 20 cm below the floor level
  if (slab?.length >= 3) {
    const pts = slab.map(([x, z]) => { const [X, Y] = toIfc(x, z); return add(`IFCCARTESIANPOINT((${real(X)},${real(Y)}))`); });
    const poly = add(`IFCPOLYLINE((${[...pts, pts[0]].map((p) => "#" + p).join(",")}))`);
    const prof = add(`IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#${poly})`);
    const solid = add(`IFCEXTRUDEDAREASOLID(#${prof},#${add(`IFCAXIS2PLACEMENT3D(#${P3(0, 0, floorY - 0.2)},$,$)`)},#${zDir},0.2)`);
    contained.push(add(`IFCSLAB('${ifcGuid()}',$,'Floor',$,$,#${place()},#${shape(solid, slabStyle)},$,.FLOOR.)`));
  }

  W.addMarkers(markers.map((mk) => ({ ...mk, desc: "Scan marker (zzikbang)", ifc: viewerToIfc(mk.pos) })));
  return W.finish();
}

/**
 * A mesh model (e.g. SketchUp STL) as IFC4: one IfcBuildingElementProxy per mesh with an
 * IfcTriangulatedFaceSet, plus marker objects. Coordinates are the model's own (viewer axes, metres).
 * @param meshes  [{name, positions: Float32Array (xyz...), index?: ArrayLike, color: [r, g, b]}]
 * @param markers [{name, tag, pos: Vector3 (model space, viewer axes)}]
 */
export function buildMeshIfc({ name, meshes, markers }) {
  const W = ifcWriter(name, "Converted from a mesh model (zzikbang)");
  const { add, style, shape, place, contained } = W;
  const styles = new Map();
  meshes.forEach((m, i) => {
    // shared vertices (welded at 0.1 mm) keep the file small
    const keyOf = (x, y, z) => `${Math.round(x * 1e4)},${Math.round(y * 1e4)},${Math.round(z * 1e4)}`;
    const ids = new Map(), coords = [], tris = [];
    const P = m.positions, count = m.index ? m.index.length : P.length / 3;
    const vid = (k) => {
      const x = P[k * 3], y = P[k * 3 + 1], z = P[k * 3 + 2];
      const key = keyOf(x, y, z);
      let id = ids.get(key);
      if (!id) { coords.push(`(${real(x)},${real(-z)},${real(y)})`); id = coords.length; ids.set(key, id); }
      return id;
    };
    for (let t = 0; t + 2 < count; t += 3) {
      const a = vid(m.index ? m.index[t] : t), b = vid(m.index ? m.index[t + 1] : t + 1), c = vid(m.index ? m.index[t + 2] : t + 2);
      if (a !== b && b !== c && a !== c) tris.push(`(${a},${b},${c})`);
    }
    if (!tris.length) return;
    const list = add(`IFCCARTESIANPOINTLIST3D((${coords.join(",")}),$)`);
    const faces = add(`IFCTRIANGULATEDFACESET(#${list},$,$,(${tris.join(",")}),$)`);
    const ck = m.color.map((v) => v.toFixed(3)).join(",");
    if (!styles.has(ck)) styles.set(ck, style(...m.color, `Colour ${styles.size + 1}`));
    contained.push(add(`IFCBUILDINGELEMENTPROXY('${ifcGuid()}',$,${str(m.name || `Model-${String(i + 1).padStart(2, "0")}`)},'mesh geometry','Model',#${place()},#${shape(faces, styles.get(ck), "Tessellation")},$,.NOTDEFINED.)`));
  });
  W.addMarkers(markers.map((mk) => ({ ...mk, desc: "Sensor marker (zzikbang)", ifc: viewerToIfc(mk.pos) })));
  return W.finish();
}

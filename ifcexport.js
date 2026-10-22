// Writes scan-detected markers back into the original IFC as marker objects, so the next scan can be
// aligned from the IFC alone. The original STEP text is kept byte-for-byte; new entities are appended
// to the DATA section:
//   IfcBuildingElementProxy "Marker-NN" (ObjectType "Marker"), a red 20 cm cube, absolute placement,
//   contained in the first building storey.
import * as THREE from "three";

const B64 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_$";

/** IFC GlobalId: 128 random bits packed into 22 characters (2 bits, then 6 bits per char) */
export function ifcGuid() {
  const b = crypto.getRandomValues(new Uint8Array(16));
  let bits = "";
  for (const x of b) bits += x.toString(2).padStart(8, "0");
  let out = B64[parseInt(bits.slice(0, 2), 2)];
  for (let i = 2; i < 128; i += 6) out += B64[parseInt(bits.slice(i, i + 6), 2)];
  return out;
}

/** metres per IFC length unit, from the project's unit assignment */
export function lengthUnitScale(text) {
  const si = text.match(/IFCSIUNIT\s*\(\s*\*\s*,\s*\.LENGTHUNIT\.\s*,\s*(\$|\.(\w+)\.)\s*,\s*\.METRE\.\s*\)/i);
  if (si) return { MILLI: 1e-3, CENTI: 1e-2, DECI: 1e-1, KILO: 1e3 }[si[2]?.toUpperCase()] ?? 1;
  if (/IFCCONVERSIONBASEDUNIT\s*\([^;]*\.LENGTHUNIT\.\s*,\s*'(FOOT|FEET)'/i.test(text)) return 0.3048;
  if (/IFCCONVERSIONBASEDUNIT\s*\([^;]*\.LENGTHUNIT\.\s*,\s*'INCH'/i.test(text)) return 0.0254;
  return 1;
}

export const real = (v) => {
  const s = (Math.abs(v) < 1e-9 ? 0 : v).toFixed(6);
  return s.includes(".") ? s : s + ".";
};
export const str = (s) => `'${String(s).replace(/'/g, "''").replace(/[^\x20-\x7e]/g, "_")}'`;

/**
 * @param text      original IFC (STEP) text
 * @param markers   [{name, tag, pos}] with pos in the viewer's model space (web-ifc output, metres)
 * @param coordMatrix web-ifc coordination matrix used when the model was loaded (column-major 16, viewer axes)
 * @returns new IFC text
 */
export function addMarkersToIfc(text, markers, { coordMatrix, size = 0.2 }) {
  const schema = (text.match(/FILE_SCHEMA\s*\(\s*\(\s*'([^']+)'/i)?.[1] || "IFC4").toUpperCase();
  const is2x3 = schema.startsWith("IFC2X3");
  const unit = lengthUnitScale(text);
  // web-ifc output = coordination * Yup * IFC, where Yup maps IFC (x, y, z) -> viewer (x, z, -y) and the
  // coordination matrix (origin shift) acts in viewer axes; undo both (in metres), then convert units
  const yUpInverse = new THREE.Matrix4().set(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1);
  const toIfc = yUpInverse.multiply(new THREE.Matrix4().fromArray(coordMatrix).invert());

  let next = 0;
  for (const m of text.matchAll(/#(\d+)\s*=/g)) next = Math.max(next, +m[1]);
  const id = () => ++next;
  const firstId = (re) => text.match(re)?.[1];
  const owner = firstId(/#(\d+)\s*=\s*IFCOWNERHISTORY\s*\(/i);
  const ctx = firstId(/#(\d+)\s*=\s*IFCGEOMETRICREPRESENTATIONSUBCONTEXT\s*\(\s*'Body'/i) || firstId(/#(\d+)\s*=\s*IFCGEOMETRICREPRESENTATIONCONTEXT\s*\(/i);
  const container = firstId(/#(\d+)\s*=\s*IFCBUILDINGSTOREY\s*\(/i) || firstId(/#(\d+)\s*=\s*IFCBUILDING\s*\(/i) || firstId(/#(\d+)\s*=\s*IFCSITE\s*\(/i);
  if (!ctx) throw new Error("IFC에 형상 컨텍스트(IfcGeometricRepresentationContext)가 없습니다");
  const ownerRef = owner ? `#${owner}` : "$";

  const lines = [];
  const add = (body) => { const n = id(); lines.push(`#${n}= ${body};`); return n; };
  // shared: red surface style and the cube's profile
  const colour = add(`IFCCOLOURRGB('Marker red',1.,0.,0.)`);
  const rendering = add(`IFCSURFACESTYLERENDERING(#${colour},0.,$,$,$,$,$,$,.NOTDEFINED.)`);
  const surf = add(`IFCSURFACESTYLE('Marker',.BOTH.,(#${rendering}))`);
  const styleRef = is2x3 ? `#${add(`IFCPRESENTATIONSTYLEASSIGNMENT((#${surf}))`)}` : `#${surf}`;
  const s = size / unit;
  const origin2 = add(`IFCCARTESIANPOINT((0.,0.))`);
  const place2 = add(`IFCAXIS2PLACEMENT2D(#${origin2},$)`);
  const profile = add(`IFCRECTANGLEPROFILEDEF(.AREA.,'Marker',#${place2},${real(s)},${real(s)})`);
  const up = add(`IFCDIRECTION((0.,0.,1.))`);
  const base = add(`IFCCARTESIANPOINT((0.,0.,${real(-s / 2)}))`); // centred on the marker point
  const solidPlace = add(`IFCAXIS2PLACEMENT3D(#${base},$,$)`);

  const proxies = [];
  markers.forEach((mk, i) => {
    const w = mk.pos.clone().applyMatrix4(toIfc).divideScalar(unit); // IFC world coordinates, IFC units
    const solid = add(`IFCEXTRUDEDAREASOLID(#${profile},#${solidPlace},#${up},${real(s)})`);
    add(`IFCSTYLEDITEM(#${solid},(${styleRef}),$)`);
    const rep = add(`IFCSHAPEREPRESENTATION(#${ctx},'Body','SweptSolid',(#${solid}))`);
    const shape = add(`IFCPRODUCTDEFINITIONSHAPE($,$,(#${rep}))`);
    const pt = add(`IFCCARTESIANPOINT((${real(w.x)},${real(w.y)},${real(w.z)}))`);
    const ax = add(`IFCAXIS2PLACEMENT3D(#${pt},$,$)`);
    const lp = add(`IFCLOCALPLACEMENT($,#${ax})`);
    const name = mk.name || `Marker-${String(i + 1).padStart(2, "0")}`;
    const last = is2x3 ? "$" : ".NOTDEFINED.";
    proxies.push(add(`IFCBUILDINGELEMENTPROXY('${ifcGuid()}',${ownerRef},${str(name)},'Scan marker (zzikbang)','Marker',#${lp},#${shape},${str(mk.tag || "")},${last})`));
  });
  if (container) add(`IFCRELCONTAINEDINSPATIALSTRUCTURE('${ifcGuid()}',${ownerRef},'Markers',$,(${proxies.map((p) => "#" + p).join(",")}),#${container})`);

  // append to the DATA section (the last ENDSEC before END-ISO-10303-21)
  const end = text.lastIndexOf("END-ISO-10303-21");
  const endsec = text.lastIndexOf("ENDSEC;", end < 0 ? text.length : end);
  if (endsec < 0) throw new Error("IFC 파일 구조(DATA 섹션)를 찾지 못했습니다");
  const nl = text.includes("\r\n") ? "\r\n" : "\n";
  return text.slice(0, endsec) + lines.join(nl) + nl + text.slice(endsec);
}

// ---------------------------------------------------------------- zip (store only)
const CRC = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; t[n] = c >>> 0; }
  return t;
})();
function crc32(bytes) {
  let c = 0xffffffff;
  for (let i = 0; i < bytes.length; i++) c = CRC[(c ^ bytes[i]) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}
/** a one-file zip archive without compression (the artifact viewer only saves allow-listed types) */
export function zipOne(filename, bytes) {
  const name = new TextEncoder().encode(filename);
  const crc = crc32(bytes), size = bytes.length;
  const local = new DataView(new ArrayBuffer(30));
  local.setUint32(0, 0x04034b50, true); local.setUint16(4, 20, true); local.setUint16(6, 0x0800, true);
  local.setUint32(14, crc, true); local.setUint32(18, size, true); local.setUint32(22, size, true); local.setUint16(26, name.length, true);
  const central = new DataView(new ArrayBuffer(46));
  central.setUint32(0, 0x02014b50, true); central.setUint16(4, 20, true); central.setUint16(6, 20, true); central.setUint16(8, 0x0800, true);
  central.setUint32(16, crc, true); central.setUint32(20, size, true); central.setUint32(24, size, true); central.setUint16(28, name.length, true);
  const end = new DataView(new ArrayBuffer(22));
  const cdOffset = 30 + name.length + size, cdSize = 46 + name.length;
  end.setUint32(0, 0x06054b50, true); end.setUint16(8, 1, true); end.setUint16(10, 1, true);
  end.setUint32(12, cdSize, true); end.setUint32(16, cdOffset, true);
  return new Blob([local, name, bytes, central, name, end], { type: "application/zip" });
}

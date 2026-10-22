// Generic 3D models (what SketchUp and other tools export: COLLADA / KMZ / OBJ / glTF / FBX / STL)
// loaded as a model layer with the same shape as an IFC layer, so opacity, floor levels, automatic
// alignment and marker finding all work on them. Textures are not used; colours come from materials.
import * as THREE from "three";
import { hsv } from "./markers.js";

export const MESH_EXT = ["dae", "kmz", "obj", "glb", "gltf", "fbx", "stl"];
// formats whose files carry their own unit and up axis
const SELF_DESCRIBING = new Set(["dae", "kmz", "glb", "gltf"]);

const manager = new THREE.LoadingManager();
// textures and side files are not available in the browser upload: do not try to fetch them
manager.setURLModifier(() => "data:,");

async function parseRoot(ext, buf) {
  const text = () => new TextDecoder().decode(buf);
  switch (ext) {
    case "glb": case "gltf": {
      const { GLTFLoader } = await import("three/addons/loaders/GLTFLoader.js");
      return (await new GLTFLoader(manager).parseAsync(ext === "glb" ? buf : text(), "")).scene;
    }
    case "dae": {
      const { ColladaLoader } = await import("three/addons/loaders/ColladaLoader.js");
      return new ColladaLoader(manager).parse(text(), "").scene; // applies <unit> and Z_UP
    }
    case "kmz": {
      const { KMZLoader } = await import("three/addons/loaders/KMZLoader.js");
      return new KMZLoader(manager).parse(buf).scene;
    }
    case "obj": {
      const { OBJLoader } = await import("three/addons/loaders/OBJLoader.js");
      return new OBJLoader(manager).parse(text());
    }
    case "fbx": {
      const { FBXLoader } = await import("three/addons/loaders/FBXLoader.js");
      return new FBXLoader(manager).parse(buf, "");
    }
    case "stl": {
      const { STLLoader } = await import("three/addons/loaders/STLLoader.js");
      const root = new THREE.Group();
      root.add(new THREE.Mesh(new STLLoader(manager).parse(buf), new THREE.MeshStandardMaterial({ color: 0xbfc5cc })));
      root.rotation.x = -Math.PI / 2; // STL is Z-up
      return root;
    }
  }
  throw new Error(`지원하지 않는 형식: ${ext}`);
}

/**
 * @param askUnit (guessMm, sizeText) => Promise<number|null> : millimetres per model unit, for formats
 *                without units (OBJ / STL / FBX)
 */
export async function loadMeshModel(file, { askUnit }) {
  const ext = file.name.split(".").pop().toLowerCase();
  const root = await parseRoot(ext, await file.arrayBuffer());
  root.updateMatrixWorld(true);
  let box = new THREE.Box3().setFromObject(root);
  if (box.isEmpty()) throw new Error("모델에서 형상을 찾지 못했습니다");
  let scale = 1;
  if (!SELF_DESCRIBING.has(ext)) {
    // guess from the size: a building is a few to a few hundred metres
    const size = box.getSize(new THREE.Vector3()), big = Math.max(size.x, size.y, size.z);
    const guessMm = big > 2000 ? 1 : big > 400 ? 10 : big > 150 ? 25.4 : 1000;
    const mm = await askUnit(guessMm, `${size.x.toFixed(1)} × ${size.y.toFixed(1)} × ${size.z.toFixed(1)} (모델 단위)`);
    if (!mm) return null;
    scale = mm / 1000;
  }

  // bake every mesh into world space, one Lambert material per source colour
  const group = new THREE.Group(), materials = [], byColor = new Map(), objects = [], pieces = [];
  const S = new THREE.Matrix4().makeScale(scale, scale, scale);
  let n = 0;
  root.traverse((o) => {
    if (!o.isMesh || !o.geometry?.attributes?.position) return;
    const mats = Array.isArray(o.material) ? o.material : [o.material];
    const src = mats[0] || {};
    const color = src.color ? src.color.clone() : new THREE.Color(0.75, 0.75, 0.75);
    const opacity = src.transparent ? src.opacity ?? 1 : 1;
    let geo = o.geometry.clone().applyMatrix4(new THREE.Matrix4().multiplyMatrices(S, o.matrixWorld));
    for (const k of Object.keys(geo.attributes)) if (k !== "position" && k !== "normal") geo.deleteAttribute(k);
    geo.clearGroups();
    // STL facet normals are often zero or wrong; rebuild them from the triangles
    if (!geo.attributes.normal || ext === "stl") geo.computeVertexNormals();
    const key = `${color.getHexString()}|${opacity.toFixed(2)}`;
    let mat = byColor.get(key);
    if (!mat) {
      mat = new THREE.MeshLambertMaterial({ color, transparent: true, side: THREE.DoubleSide });
      mat.userData.baseColor = color.clone();
      mat.userData.baseOpacity = opacity;
      byColor.set(key, mat);
      materials.push(mat);
    }
    group.add(new THREE.Mesh(geo, mat));
    // object identity for marker objects: node or parent component name (SketchUp component names)
    geo.computeBoundingBox();
    const b = geo.boundingBox, size = b.getSize(new THREE.Vector3());
    const name = o.name || o.parent?.name || "";
    objects.push({ id: ++n, type: ext.toUpperCase(), name, objectType: o.parent?.name || "", desc: "", tag: "", color: [color.r, color.g, color.b], center: b.getCenter(new THREE.Vector3()), size });
    const [, sat] = hsv(color.r, color.g, color.b);
    if (sat > 0.3 && Math.max(size.x, size.y, size.z) < 3) {
      pieces.push({ color: [color.r, color.g, color.b], verts: Float32Array.from(geo.attributes.position.array), box: b.clone() });
    }
  });
  if (!group.children.length) throw new Error("모델에서 형상을 찾지 못했습니다");
  return {
    group, materials, objects, pieces, kind: ext,
    coordMatrix: new THREE.Matrix4().toArray(),
    info: `${ext.toUpperCase()} 메시 ${group.children.length.toLocaleString()}개${scale !== 1 ? ` · 1 단위 = ${+(scale * 1000).toFixed(3)} mm` : ""}`,
  };
}
